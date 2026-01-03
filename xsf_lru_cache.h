#ifndef XSF_LRU_CACHE_H
#define XSF_LRU_CACHE_H

#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "xsf_cache.h"

namespace xsf_simple_cache {

template <typename K, typename V, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFLruCache : public XSFCache<K, V> {
   public:
    explicit XSFLruCache(size_t capacity, Hash hash = Hash{},
                         KeyEqual key_equal = KeyEqual{})
        : capacity_(capacity), key2node_(0, hash, key_equal) {}

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
    std::mutex mutex_;
};

template <typename K, typename V, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFLruCacheCreator : public XSFCacheCreator<K, V> {
   public:
    explicit XSFLruCacheCreator(Hash hash = Hash{}, KeyEqual eq = KeyEqual{})
        : hash_(std::move(hash)), key_equal_(std::move(eq)) {}

    std::unique_ptr<XSFCache<K, V>> create(size_t capacity) const override {
        return std::make_unique<XSFLruCache<K, V, Hash, KeyEqual>>(
            capacity, hash_, key_equal_);
    }

   private:
    Hash hash_;
    KeyEqual key_equal_;
};

}  // namespace xsf_simple_cache

#endif  // XSF_LRU_CACHE_H