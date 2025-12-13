#ifndef XSF_LRU_CACHE_H
#define XSF_LRU_CACHE_H

#include <list>
#include <mutex>
#include <unordered_map>

#include "xsf_cache.h"

namespace xsf_simple_cache {

template <typename K, typename V>
class XSFLruCache : public XSFCache<K, V> {
   public:
    XSFLruCache(size_t capacity) : capacity_(capacity) {}

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
        // 移除链表头部节点
        nodes_.pop_front();
        // 移除映射
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
    std::unordered_map<K, typename std::list<Node>::iterator> key2node_;
    std::mutex mutex_;
};

}  // namespace xsf_simple_cache

#endif  // XSF_LRU_CACHE_H