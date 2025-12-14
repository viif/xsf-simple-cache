#ifndef XSF_LRU_K_CACHE_H
#define XSF_LRU_K_CACHE_H

#include <list>
#include <mutex>
#include <unordered_map>

#include "xsf_cache.h"

namespace xsf_simple_cache {

// 非线程安全的基础 LRU 缓存实现
template <typename K, typename V>
class XSFLruCacheUnsafe : public XSFCache<K, V> {
   public:
    XSFLruCacheUnsafe(size_t capacity) : capacity_(capacity) {}

    void put(const K& key, const V& value) override {
        if (capacity_ == 0) {
            return;
        }
        if (contains(key)) {
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
        if (!contains(key)) {
            return std::nullopt;
        }
        makeRecently(key);
        return key2node_[key]->value;
    }

    bool contains(const K& key) { return key2node_.count(key) != 0; }

    void remove(const K& key) {
        if (key2node_.count(key) == 0) {
            return;
        }
        auto node_iter = key2node_[key];
        nodes_.erase(node_iter);
        key2node_.erase(key);
    }

    void pop() {
        if (nodes_.empty()) {
            return;
        }
        popLeastRecently();
    }

    bool empty() const { return key2node_.empty(); }

    size_t size() const { return key2node_.size(); }

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
};

template <typename K, typename V>
class XSFLruKCache : public XSFCache<K, V> {
   public:
    XSFLruKCache(size_t capacity, size_t k)
        : k_(k),
          capacity_(capacity),
          history_lru_(capacity),
          cache_lru_(capacity) {}

    void put(const K& key, const V& value) override {
        if (capacity_ == 0) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        // 检查缓存队列
        if (cache_lru_.contains(key)) {
            // key 已存在，更新
            cache_lru_.put(key, value);
            return;
        }

        // 检查历史队列
        auto record_opt = history_lru_.get(key);
        if (record_opt.has_value()) {
            // key 已存在，更新访问次数和数值
            auto record = record_opt.value();
            // 如果访问次数达到 k，迁移到缓存队列
            if (record.count + 1 >= k_) {
                history_lru_.remove(key);
                cache_lru_.put(key, value);
            } else {
                record.count += 1;
                record.value = value;
                history_lru_.put(key, record);
            }
        } else {
            // key 不存在，插入历史队列
            if (size() >= capacity_) {
                popLeastRecently();
            }
            AccessRecord new_record{1, value};
            history_lru_.put(key, new_record);
        }
    }

    std::optional<V> get(const K& key) override {
        std::lock_guard<std::mutex> lock(mutex_);
        // 检查缓存队列
        auto value_opt = cache_lru_.get(key);
        if (value_opt.has_value()) {
            return value_opt;
        }

        // 检查历史队列
        auto record_opt = history_lru_.get(key);
        if (record_opt.has_value()) {
            auto record = record_opt.value();
            // 如果访问次数达到 k，迁移到缓存队列
            if (record.count + 1 >= k_) {
                history_lru_.remove(key);
                cache_lru_.put(key, record.value);
                return record.value;
            } else {
                record.count += 1;
                history_lru_.put(key, record);
                return record.value;
            }
        }

        return std::nullopt;
    }

   private:
    void popLeastRecently() {
        if (!history_lru_.empty()) {
            history_lru_.pop();
        } else {
            cache_lru_.pop();
        }
    }

    size_t size() const { return history_lru_.size() + cache_lru_.size(); }

    const size_t k_;  // 访问次数阈值
    const size_t capacity_;
    std::mutex mutex_;

    struct AccessRecord {
        size_t count;
        V value;
    };

    XSFLruCacheUnsafe<K, AccessRecord> history_lru_;  // 访问次数小于 k 的记录
    XSFLruCacheUnsafe<K, V> cache_lru_;               // 访问次数达到 k 的记录
};

}  // namespace xsf_simple_cache

#endif  // XSF_LRU_K_CACHE_H