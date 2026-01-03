#ifndef XSF_ARC_CACHE_H
#define XSF_ARC_CACHE_H

#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "xsf_cache.h"

namespace xsf_simple_cache {

template <typename K, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFArcGhostList {
   public:
    explicit XSFArcGhostList(size_t capacity, Hash hash = Hash{},
                             KeyEqual key_equal = KeyEqual{})
        : capacity_(capacity), key2key_(0, hash, key_equal) {}

    void put(const K& key) {
        if (isZeroCapacity()) {
            return;
        }
        ensureCapacity();
        updateKeyPosition(key);
    }

    bool contains(const K& key) const { return key2key_.count(key) != 0; }

    void remove(const K& key) {
        if (!contains(key)) {
            return;
        }
        eraseKey(key);
    }

   private:
    bool isZeroCapacity() const { return capacity_ == 0; }

    void ensureCapacity() {
        if (keys_.size() >= capacity_) {
            evictOldest();
        }
    }

    void updateKeyPosition(const K& key) {
        if (contains(key)) {
            moveKeyToBack(key);
        } else {
            insertNewKey(key);
        }
    }

    void evictOldest() {
        const K& old_key = keys_.front();
        eraseFromMap(old_key);
        keys_.pop_front();
    }

    void moveKeyToBack(const K& key) {
        auto it = key2key_[key];
        keys_.erase(it);
        insertKeyToList(key);
    }

    void insertNewKey(const K& key) {
        insertKeyToList(key);
        key2key_[key] = std::prev(keys_.end());
    }

    void insertKeyToList(const K& key) {
        keys_.push_back(key);
        key2key_[key] = std::prev(keys_.end());
    }

    void eraseFromMap(const K& key) { key2key_.erase(key); }

    void eraseKey(const K& key) {
        auto it = key2key_[key];
        keys_.erase(it);
        eraseFromMap(key);
    }

    const size_t capacity_;
    std::list<K> keys_;
    std::unordered_map<K, typename std::list<K>::iterator, Hash, KeyEqual>
        key2key_;
};

template <typename K, typename V, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFArcLruList {
   public:
    explicit XSFArcLruList(size_t capacity, Hash hash = Hash{},
                           KeyEqual key_equal = KeyEqual{})
        : capacity_(capacity),
          key2node_(0, hash, key_equal),
          ghost_list_(capacity, hash, key_equal) {}

    void put(const K& key, const V& value) {
        if (IsZeroCapacity()) {
            return;
        }
        handlePutOperation(key, value);
    }

    std::optional<V> get(const K& key) {
        if (IsZeroCapacity()) {
            return std::nullopt;
        }
        return handleGetOperation(key);
    }

    bool contains(const K& key) const { return key2node_.count(key) != 0; }

    uint8_t getCountByKey(const K& key) const {
        if (!contains(key)) {
            return 0;
        }
        return key2node_.at(key)->count;
    }

    void increaseCapacity() { capacity_++; }

    void decreaseCapacity() {
        if (IsZeroCapacity()) {
            return;
        }
        capacity_--;
        ensureCapacityForInsertion();
    }

    bool containsInGhostList(const K& key) const {
        return ghost_list_.contains(key);
    }

    void removeInGhostList(const K& key) { ghost_list_.remove(key); }

   private:
    bool IsZeroCapacity() const { return capacity_ == 0; }

    void handlePutOperation(const K& key, const V& value) {
        if (isKeyPresent(key)) {
            updateExistingKey(key, value);
        } else {
            insertNewKey(key, value);
        }
    }

    std::optional<V> handleGetOperation(const K& key) {
        if (!isKeyPresent(key)) {
            return std::nullopt;
        }
        return retrieveAndPromoteValue(key);
    }

    bool isKeyPresent(const K& key) const { return key2node_.count(key) != 0; }

    void updateExistingKey(const K& key, const V& value) {
        updateNodeValue(key, value);
        promoteNodeToRecent(key);
    }

    void insertNewKey(const K& key, const V& value) {
        ensureCapacityForInsertion();
        addNewNode(key, value);
    }

    std::optional<V> retrieveAndPromoteValue(const K& key) {
        promoteNodeToRecent(key);
        return getValueByKey(key);
    }

    void ensureCapacityForInsertion() {
        if (isAtFullCapacity()) {
            evictLeastRecentlyUsed();
        }
    }

    bool isAtFullCapacity() const { return key2node_.size() >= capacity_; }

    V getValueByKey(const K& key) { return key2node_[key]->value; }

    void promoteNodeToRecent(const K& key) {
        key2node_[key]->count++;
        auto& node_iter = key2node_[key];
        nodes_.splice(nodes_.end(), nodes_, node_iter);
    }

    void evictLeastRecentlyUsed() {
        const K& lru_key = nodes_.front().key;
        ghost_list_.put(lru_key);
        key2node_.erase(lru_key);
        nodes_.pop_front();
    }

    void addNewNode(const K& key, const V& value) {
        nodes_.emplace_back(key, value);
        key2node_[key] = std::prev(nodes_.end());
    }

    void updateNodeValue(const K& key, const V& value) {
        key2node_[key]->value = value;
    }

    struct Node {
        K key;
        V value;
        uint8_t count;

        Node() = default;
        Node(const K& k, const V& v) : key(k), value(v), count(1) {}
    };
    size_t capacity_;

    std::list<Node> nodes_;
    std::unordered_map<K, typename std::list<Node>::iterator, Hash, KeyEqual>
        key2node_;
    XSFArcGhostList<K, Hash, KeyEqual> ghost_list_;
};

template <typename K, typename V, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFArcLfuList {
   public:
    explicit XSFArcLfuList(size_t capacity, Hash hash = Hash{},
                           KeyEqual key_equal = KeyEqual{})
        : capacity_(capacity),
          key2freq_(0, hash, key_equal),
          key2node_(0, hash, key_equal),
          ghost_list_(capacity, hash, key_equal) {}

    void put(const K& key, const V& value) {
        if (IsZeroCapacity()) {
            return;
        }
        handlePutOperation(key, value);
    }

    std::optional<V> get(const K& key) {
        if (IsZeroCapacity()) {
            return std::nullopt;
        }
        return handleGetOperation(key);
    }

    bool contains(const K& key) const { return key2node_.count(key) != 0; }

    void increaseCapacity() { capacity_++; }

    void decreaseCapacity() {
        if (IsZeroCapacity()) {
            return;
        }
        capacity_--;
        ensureCapacityAfterDecrease();
    }

    bool containsInGhostList(const K& key) const {
        return ghost_list_.contains(key);
    }

    void removeInGhostList(const K& key) { ghost_list_.remove(key); }

   private:
    struct Node {
        K key;
        V value;

        Node() = default;
        Node(const K& k, const V& v) : key(k), value(v) {}
    };

    bool IsZeroCapacity() const { return capacity_ == 0; }

    void handlePutOperation(const K& key, const V& value) {
        if (isKeyPresent(key)) {
            updateExistingKey(key, value);
        } else {
            insertNewKey(key, value);
        }
    }

    std::optional<V> handleGetOperation(const K& key) {
        if (!isKeyPresent(key)) {
            return std::nullopt;
        }
        return retrieveAndIncreaseFreq(key);
    }

    bool isKeyPresent(const K& key) const { return key2node_.count(key) != 0; }

    void updateExistingKey(const K& key, const V& value) {
        updateNodeValue(key, value);
        increaseKeyFrequency(key);
    }

    void insertNewKey(const K& key, const V& value) {
        ensureCapacityForInsertion();
        addNewNodeWithFrequencyOne(key, value);
    }

    std::optional<V> retrieveAndIncreaseFreq(const K& key) {
        increaseKeyFrequency(key);
        return getNodeValue(key);
    }

    V getNodeValue(const K& key) { return key2node_[key]->value; }

    void updateNodeValue(const K& key, const V& value) {
        key2node_[key]->value = value;
    }

    void increaseKeyFrequency(const K& key) {
        uint8_t old_freq = key2freq_[key]++;

        moveNodeToNewFrequency(key, old_freq, old_freq + 1);
        auditEmptyFrequencyList(old_freq);
    }

    void auditEmptyFrequencyList(uint8_t freq) {
        cleanupEmptyFrequencyList(freq);
        updateMinFrequencyIfNeeded(freq);
    }

    void ensureCapacityForInsertion() {
        if (isAtFullCapacity()) {
            removeLeastFrequentlyUsed();
        }
    }

    bool isAtFullCapacity() const { return key2node_.size() >= capacity_; }

    void addNewNodeWithFrequencyOne(const K& key, const V& value) {
        key2freq_[key] = 1;
        min_freq_ = 1;
        freq2nodes_[1].emplace_back(key, value);
        key2node_[key] = std::prev(freq2nodes_[1].end());
    }

    void ensureCapacityAfterDecrease() {
        if (isOverCapacity()) {
            removeLeastFrequentlyUsed();
        }
    }

    bool isOverCapacity() const { return key2node_.size() > capacity_; }

    void removeLeastFrequentlyUsed() {
        removeLfuNode();
        auditEmptyFrequencyList(min_freq_);
    }

    void removeLfuNode() {
        auto& nodes = freq2nodes_[min_freq_];
        const K& key_to_remove = nodes.front().key;
        ghost_list_.put(key_to_remove);
        key2node_.erase(key_to_remove);
        key2freq_.erase(key_to_remove);
        nodes.pop_front();
    }

    void moveNodeToNewFrequency(const K& key, uint8_t old_freq,
                                uint8_t new_freq) {
        auto& old_freq_list = freq2nodes_[old_freq];
        auto& new_freq_list = freq2nodes_[new_freq];
        auto node_iter = key2node_[key];
        new_freq_list.splice(new_freq_list.end(), old_freq_list, node_iter);
        key2node_[key] = std::prev(new_freq_list.end());
    }

    void cleanupEmptyFrequencyList(uint8_t freq) {
        if (freq2nodes_[freq].empty()) {
            freq2nodes_.erase(freq);
        }
    }

    void updateMinFrequencyIfNeeded(uint8_t freq) {
        if (freq == min_freq_ && freq2nodes_.count(freq) == 0) {
            min_freq_ = freq2nodes_.begin()->first;
        }
    }

    size_t capacity_;
    uint8_t min_freq_{0};

    std::unordered_map<K, uint8_t, Hash, KeyEqual> key2freq_;
    std::map<uint8_t, std::list<Node>> freq2nodes_;
    std::unordered_map<K, typename std::list<Node>::iterator, Hash, KeyEqual>
        key2node_;
    XSFArcGhostList<K, Hash, KeyEqual> ghost_list_;
};

template <typename K, typename V, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFArcCache : public XSFCache<K, V> {
   public:
    static constexpr uint8_t DEFAULT_K = 3;

    explicit XSFArcCache(size_t capacity, uint8_t k = DEFAULT_K,
                         Hash hash = Hash{}, KeyEqual key_equal = KeyEqual{})
        : capacity_(capacity),
          k_(k),
          lru_list_(capacity - capacity / 2, hash, key_equal),
          lfu_list_(capacity / 2, hash, key_equal) {}

    void put(const K& key, const V& value) override {
        if (IsZeroCapacity()) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        handlePutOperation(key, value);
    }

    std::optional<V> get(const K& key) override {
        if (IsZeroCapacity()) {
            return std::nullopt;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        return handleGetOperation(key);
    }

   private:
    enum class KeyLocation { IN_LRU, IN_LFU, NOT_IN_CACHE };

    bool IsZeroCapacity() const { return capacity_ == 0; }

    void handlePutOperation(const K& key, const V& value) {
        KeyLocation location = locateKey(key);
        adjustCacheByGhostList(location, key);
        updateCacheForKey(key, value, location);
    }

    std::optional<V> handleGetOperation(const K& key) {
        KeyLocation location = locateKey(key);
        adjustCacheByGhostList(location, key);
        return retrieveValueForKey(key, location);
    }

    KeyLocation locateKey(const K& key) const {
        if (lru_list_.contains(key)) {
            return KeyLocation::IN_LRU;
        }
        if (lfu_list_.contains(key)) {
            return KeyLocation::IN_LFU;
        }
        return KeyLocation::NOT_IN_CACHE;
    }

    void adjustCacheByGhostList(KeyLocation location, const K& key) {
        if (location != KeyLocation::NOT_IN_CACHE) {
            return;  // 键已在缓存中，无需调整
        }

        if (isKeyInLruGhostList(key)) {
            adjustForLruGhostHit(key);
        } else if (isKeyInLfuGhostList(key)) {
            adjustForLfuGhostHit(key);
        }
    }

    void updateCacheForKey(const K& key, const V& value, KeyLocation location) {
        updateLruListForKey(key, value);
        updateLfuListForKey(key, value, location);
        migrateToLfuListIfNeeded(key, value);
    }

    std::optional<V> retrieveValueForKey(const K& key, KeyLocation location) {
        if (location == KeyLocation::IN_LRU) {
            return retrieveFromLruList(key);
        } else if (location == KeyLocation::IN_LFU) {
            return retrieveFromLfuList(key);
        }
        return std::nullopt;
    }

    bool isKeyInLruGhostList(const K& key) const {
        return lru_list_.containsInGhostList(key);
    }

    bool isKeyInLfuGhostList(const K& key) const {
        return lfu_list_.containsInGhostList(key);
    }

    void adjustForLruGhostHit(const K& key) {
        lru_list_.removeInGhostList(key);
        lru_list_.increaseCapacity();
        lfu_list_.decreaseCapacity();
    }

    void adjustForLfuGhostHit(const K& key) {
        lfu_list_.removeInGhostList(key);
        lfu_list_.increaseCapacity();
        lru_list_.decreaseCapacity();
    }

    void updateLruListForKey(const K& key, const V& value) {
        lru_list_.put(key, value);
    }

    void updateLfuListForKey(const K& key, const V& value,
                             KeyLocation location) {
        if (location == KeyLocation::IN_LFU) {
            lfu_list_.put(key, value);
        }
    }

    void migrateToLfuListIfNeeded(const K& key, const V& value) {
        if (shouldMigrateToLfu(key)) {
            lfu_list_.put(key, value);
        }
    }

    bool shouldMigrateToLfu(const K& key) const {
        return lru_list_.getCountByKey(key) >= k_;
    }

    std::optional<V> retrieveFromLruList(const K& key) {
        auto value = lru_list_.get(key).value();
        handleLruRetrieval(key, value);
        return value;
    }

    std::optional<V> retrieveFromLfuList(const K& key) {
        return lfu_list_.get(key);
    }

    void handleLruRetrieval(const K& key, const V& value) {
        if (shouldMigrateToLfu(key)) {
            lfu_list_.put(key, value);
        }
    }

    const size_t capacity_;
    const uint8_t k_{DEFAULT_K};
    XSFArcLruList<K, V, Hash, KeyEqual> lru_list_;
    XSFArcLfuList<K, V, Hash, KeyEqual> lfu_list_;
    std::mutex mutex_;
};

template <typename K, typename V, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFArcCacheCreator : public XSFCacheCreator<K, V> {
   public:
    static constexpr uint8_t DEFAULT_K = 3;

    explicit XSFArcCacheCreator(uint8_t k = DEFAULT_K, Hash hash = Hash{},
                                KeyEqual key_equal = KeyEqual{})
        : k_(k), hash_(hash), key_equal_(key_equal) {}

    std::unique_ptr<XSFCache<K, V>> create(size_t capacity) const override {
        return std::make_unique<XSFArcCache<K, V, Hash, KeyEqual>>(
            capacity, k_, hash_, key_equal_);
    }

    uint8_t k_{DEFAULT_K};

   private:
    const Hash hash_;
    const KeyEqual key_equal_;
};

}  // namespace xsf_simple_cache

#endif  // XSF_ARC_CACHE_H