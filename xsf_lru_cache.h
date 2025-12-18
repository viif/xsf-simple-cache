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
        if (capacity_ == 0) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (key2node_.count(key) != 0) {
            // key 已存在，更新
            key2node_[key]->value = value;
            makeRecently(key);
        } else {
            // key 不存在，插入
            if (key2node_.size() >= capacity_) {
                popLeastRecently();
            }
            nodes_.emplace_back(key, value);
            key2node_[key] = std::prev(nodes_.end());
        }
    }

    std::optional<V> get(const K& key) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (key2node_.count(key) == 0) {
            return std::nullopt;
        }
        makeRecently(key);
        return key2node_[key]->value;
    }

   private:
    void makeRecently(const K& key) {
        // 移至链表后端
        auto& node_iter = key2node_[key];
        nodes_.splice(nodes_.end(), nodes_, node_iter);
    }

    void popLeastRecently() {
        // 记录将逐出的 key
        const K& key = nodes_.front().key;
        // 移除映射
        key2node_.erase(key);
        // 移除链表头部节点
        nodes_.pop_front();
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