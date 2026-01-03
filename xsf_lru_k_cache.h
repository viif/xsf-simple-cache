#ifndef XSF_LRU_K_CACHE_H
#define XSF_LRU_K_CACHE_H

#include <list>
#include <mutex>
#include <unordered_map>

#include "xsf_cache.h"

namespace xsf_simple_cache {

template <typename K, typename V, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFLruCacheUnsafe : public XSFCache<K, V> {
   public:
    explicit XSFLruCacheUnsafe(size_t capacity, Hash hash = Hash{},
                               KeyEqual key_equal = KeyEqual{})
        : capacity_(capacity), key2node_(0, hash, key_equal) {}

    void put(const K& key, const V& value) override {
        if (IsZeroCapacity()) {
            return;
        }
        handlePutOperation(key, value);
    }

    std::optional<V> get(const K& key) override {
        if (IsZeroCapacity()) {
            return std::nullopt;
        }
        return handleGetOperation(key);
    }

    bool contains(const K& key) const { return isKeyPresent(key); }

    bool empty() const { return key2node_.empty(); }

    size_t size() const { return key2node_.size(); }

    void pop() {
        if (IsZeroCapacity() || empty()) {
            return;
        }
        evictLeastRecentlyUsed();
    }

    void remove(const K& key) {
        if (IsZeroCapacity() || !isKeyPresent(key)) {
            return;
        }
        removeExistingNodeByKey(key);
    }

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
        auto& node_iter = key2node_[key];
        nodes_.splice(nodes_.end(), nodes_, node_iter);
    }

    void evictLeastRecentlyUsed() {
        const K& lru_key = nodes_.front().key;
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

    void removeExistingNodeByKey(const K& key) {
        auto& node_iter = key2node_[key];
        nodes_.erase(node_iter);
        key2node_.erase(key);
    }

    struct Node {
        K key;
        V value;

        Node() = default;
        Node(const K& k, const V& v) : key(k), value(v) {}
    };
    const size_t capacity_;

    std::list<Node> nodes_;
    std::unordered_map<K, typename std::list<Node>::iterator, Hash, KeyEqual>
        key2node_;
};

template <typename K, typename V, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFLruKCache : public XSFCache<K, V> {
   public:
    static constexpr uint8_t DEFAULT_K = 2;

    explicit XSFLruKCache(size_t capacity, uint8_t k = DEFAULT_K,
                          Hash hash = Hash{}, KeyEqual key_equal = KeyEqual{})
        : k_(k),
          capacity_(capacity),
          history_lru_(capacity, hash, key_equal),
          cache_lru_(capacity, hash, key_equal) {}

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
        return processGetOperation(key);
    }

   private:
    struct AccessRecord {
        uint8_t count;
        V value;
    };

    bool IsZeroCapacity() const { return capacity_ == 0; }

    void handlePutOperation(const K& key, const V& value) {
        if (isKeyInCache(key)) {
            updateKeyInCache(key, value);
        } else {
            handleKeyNotInCache(key, value);
        }
    }

    std::optional<V> processGetOperation(const K& key) {
        if (auto cache_value = tryGetFromCache(key)) {
            return cache_value;
        }
        return tryGetFromHistory(key);
    }

    bool isKeyInCache(const K& key) { return cache_lru_.contains(key); }

    void updateKeyInCache(const K& key, const V& value) {
        cache_lru_.put(key, value);
    }

    void handleKeyNotInCache(const K& key, const V& value) {
        if (auto record = history_lru_.get(key)) {
            processExistingHistoryRecordForPut(key, value, record.value());
        } else {
            insertNewKeyIntoHistory(key, value);
        }
    }

    void processExistingHistoryRecordForPut(const K& key, const V& value,
                                            AccessRecord& record) {
        if (shouldPromoteToCache(record)) {
            promoteFromHistoryToCache(key, value);
        } else {
            updateHistoryRecord(key, value, record);
        }
    }

    void insertNewKeyIntoHistory(const K& key, const V& value) {
        ensureCapacityForInsertion();
        AccessRecord record{1, value};
        history_lru_.put(key, record);
    }

    bool shouldPromoteToCache(const AccessRecord& record) {
        return record.count + 1 >= k_;
    }

    void promoteFromHistoryToCache(const K& key, const V& value) {
        history_lru_.remove(key);
        cache_lru_.put(key, value);
    }

    void updateHistoryRecord(const K& key, const V& value,
                             AccessRecord& record) {
        record.count += 1;
        record.value = value;
        history_lru_.put(key, record);
    }

    void ensureCapacityForInsertion() {
        if (isAtFullCapacity()) {
            evictLeastRecentlyUsed();
        }
    }

    bool isAtFullCapacity() const { return size() >= capacity_; }

    void evictLeastRecentlyUsed() {
        if (!history_lru_.empty()) {
            history_lru_.pop();
        } else {
            cache_lru_.pop();
        }
    }

    size_t size() const { return history_lru_.size() + cache_lru_.size(); }

    std::optional<V> tryGetFromCache(const K& key) {
        return cache_lru_.get(key);
    }

    std::optional<V> tryGetFromHistory(const K& key) {
        auto record_opt = history_lru_.get(key);
        if (!record_opt) {
            return std::nullopt;
        }
        return processHistoryHit(key, record_opt.value());
    }

    std::optional<V> processHistoryHit(const K& key, AccessRecord& record) {
        if (shouldPromoteToCache(record)) {
            return promoteAndGetFromCache(key, record);
        } else {
            return incrementAndGetFromHistory(key, record);
        }
    }

    std::optional<V> promoteAndGetFromCache(const K& key,
                                            AccessRecord& record) {
        promoteFromHistoryToCache(key, record.value);
        return record.value;
    }

    std::optional<V> incrementAndGetFromHistory(const K& key,
                                                AccessRecord& record) {
        record.count += 1;
        history_lru_.put(key, record);
        return record.value;
    }

    XSFLruCacheUnsafe<K, AccessRecord, Hash, KeyEqual>
        history_lru_;  // 访问次数小于 k 的记录
    XSFLruCacheUnsafe<K, V, Hash, KeyEqual>
        cache_lru_;  // 访问次数达到 k 的记录

    const uint8_t k_;  // 访问次数阈值
    const size_t capacity_;
    std::mutex mutex_;
};

template <typename K, typename V, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFLruKCacheCreator : public XSFCacheCreator<K, V> {
   public:
    using cache_type = XSFLruKCache<K, V, Hash, KeyEqual>;

    explicit XSFLruKCacheCreator(Hash hash = Hash{}, KeyEqual eq = KeyEqual(),
                                 uint8_t k = cache_type::DEFAULT_K)
        : hash_(std::move(hash)), key_equal_(std::move(eq)), k_(k) {}

    std::unique_ptr<XSFCache<K, V>> create(size_t capacity) const override {
        return std::make_unique<cache_type>(capacity, k_, hash_, key_equal_);
    }

    uint8_t k_{cache_type::DEFAULT_K};

   private:
    Hash hash_;
    KeyEqual key_equal_;
};

}  // namespace xsf_simple_cache

#endif  // XSF_LRU_K_CACHE_H