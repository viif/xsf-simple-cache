#ifndef XSF_LFU_AGING_CACHE_H
#define XSF_LFU_AGING_CACHE_H

#include <list>
#include <map>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "xsf_cache.h"

namespace xsf_simple_cache {

template <typename K, typename V, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFLfuAgingCache : public XSFCache<K, V> {
   public:
    static constexpr uint8_t DEFAULT_AGING_THRESHOLD = 10;

    explicit XSFLfuAgingCache(size_t capacity,
                              uint8_t aging_threshold = DEFAULT_AGING_THRESHOLD,
                              Hash hash = Hash{},
                              KeyEqual key_equal = KeyEqual{})
        : capacity_(capacity),
          aging_threshold_(aging_threshold),
          key2freq_(0, hash, key_equal),
          key2node_(0, hash, key_equal) {}

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
        updateAvgFreq(1);
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
        updateAvgFreq(old_freq + 1);
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

    void addNewNodeWithFrequencyOne(const K& key, const V& value) {
        key2freq_[key] = 1;
        min_freq_ = 1;
        freq2nodes_[1].emplace_back(key, value);
        key2node_[key] = std::prev(freq2nodes_[1].end());
    }

    bool isAtFullCapacity() const { return key2node_.size() >= capacity_; }

    void removeLeastFrequentlyUsed() {
        removeLruNode();
        auditEmptyFrequencyList(min_freq_);
    }

    void removeLruNode() {
        auto& nodes = freq2nodes_[min_freq_];
        const K& key_to_remove = nodes.front().key;
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

    void updateAvgFreq(uint8_t new_freq) {
        if (key2freq_.empty()) {
            avg_freq_ = 0;
            return;
        }

        calculateAndSetNewAvgFreq(new_freq);

        if (shouldPerformAging()) {
            performFrequencyAging();
        }
    }

    void calculateAndSetNewAvgFreq(uint8_t new_freq) {
        uint32_t total_freq = avg_freq_ * (key2freq_.size() - 1) + new_freq;
        avg_freq_ = total_freq / key2freq_.size();
    }

    bool shouldPerformAging() const { return avg_freq_ >= aging_threshold_; }

    void performFrequencyAging() {
        uint8_t decay = calculateDecayAmount();
        AgeAllFrequencies(decay);
        rebuildFrequencyStructures();
    }

    uint8_t calculateDecayAmount() const { return aging_threshold_ / 2; }

    void AgeAllFrequencies(uint8_t decay) {
        uint32_t new_total_freq = 0;

        for (auto& [k, f] : key2freq_) {
            f = applyDecayToFrequency(f, decay);
            updateMinFrequency(f);
            new_total_freq += f;
        }

        avg_freq_ = new_total_freq / key2freq_.size();
    }

    uint8_t applyDecayToFrequency(uint8_t freq, uint8_t decay) {
        if (freq > decay) {
            return freq - decay;
        } else {
            return 1;
        }
    }

    void updateMinFrequency(uint8_t new_freq) {
        min_freq_ = std::min(min_freq_, new_freq);
    }

    void rebuildFrequencyStructures() {
        auto items = collectAllCacheItems();
        clearAllFrequencyStructures();
        rebuildFromItems(items);
    }

    std::vector<std::pair<K, V>> collectAllCacheItems() {
        std::vector<std::pair<K, V>> items;
        items.reserve(key2node_.size());

        for (const auto& [k, it] : key2node_) {
            items.emplace_back(k, it->value);
        }

        return items;
    }

    void clearAllFrequencyStructures() {
        freq2nodes_.clear();
        key2node_.clear();
    }

    void rebuildFromItems(const std::vector<std::pair<K, V>>& items) {
        for (const auto& [k, v] : items) {
            uint8_t f = key2freq_[k];
            freq2nodes_[f].emplace_back(k, v);
            key2node_[k] = std::prev(freq2nodes_[f].end());
        }
    }

    const size_t capacity_;
    const uint8_t aging_threshold_;
    uint8_t min_freq_{0};
    uint8_t avg_freq_{0};
    std::mutex mutex_;

    std::unordered_map<K, uint8_t, Hash, KeyEqual> key2freq_;
    std::map<uint8_t, std::list<Node>> freq2nodes_;
    std::unordered_map<K, typename std::list<Node>::iterator, Hash, KeyEqual>
        key2node_;
};

template <typename K, typename V, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFLfuAgingCacheCreator : public XSFCacheCreator<K, V> {
   public:
    using cache_type = XSFLfuAgingCache<K, V, Hash, KeyEqual>;

    explicit XSFLfuAgingCacheCreator(
        Hash hash = Hash{}, KeyEqual eq = KeyEqual(),
        uint8_t aging_threshold = cache_type::DEFAULT_AGING_THRESHOLD)
        : hash_(std::move(hash)),
          key_equal_(std::move(eq)),
          aging_threshold_(aging_threshold) {}

    std::unique_ptr<XSFCache<K, V>> create(size_t capacity) const override {
        return std::make_unique<cache_type>(capacity, aging_threshold_, hash_,
                                            key_equal_);
    }

    uint8_t aging_threshold_{cache_type::DEFAULT_AGING_THRESHOLD};

   private:
    Hash hash_;
    KeyEqual key_equal_;
};

}  // namespace xsf_simple_cache

#endif  // XSF_LFU_AGING_CACHE_H