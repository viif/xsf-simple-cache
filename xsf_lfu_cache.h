#ifndef XSF_LFU_CACHE_H
#define XSF_LFU_CACHE_H

#include <list>
#include <mutex>
#include <unordered_map>

#include "xsf_cache.h"

namespace xsf_simple_cache {

template <typename K, typename V>
class XSFLfuCache {
   public:
    explicit XSFLfuCache(size_t capacity) : capacity_(capacity) {}

    void put(const K& key, const V& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (capacity_ == 0) {
            return;
        }
        if (key2node_.count(key) != 0) {
            // key 已存在，更新
            key2node_[key]->value = value;
            increaseFreq(key);
        } else {
            // key 不存在，加入
            if (key2node_.size() >= capacity_) {
                removeMinFreqKey();
            }
            // 新加入，即对应频率、最小频率为 1
            key2freq_[key] = 1;
            min_freq_ = 1;
            // 加入到对应频率的节点链表中
            freq2nodes_[1].emplace_back(key, value);
            // 记录 key 到节点的映射
            key2node_[key] = std::prev(freq2nodes_[1].end());
        }
    }

    std::optional<V> get(const K& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (key2node_.count(key) == 0) {
            return std::nullopt;
        }
        increaseFreq(key);
        return key2node_[key]->value;
    }

   private:
    void increaseFreq(const K& key) {
        // 找到 key 对应的频率
        uint32_t freq = key2freq_[key];
        // 更新 key 到频率的映射
        key2freq_[key]++;

        // 在新频率对应的节点链表中插入新节点
        auto& new_nodes = freq2nodes_[freq + 1];
        new_nodes.emplace_back(key, key2node_[key]->value);
        // 删除原频率对应的节点链表中的节点
        auto& old_nodes = freq2nodes_[freq];
        old_nodes.erase(key2node_[key]);
        if (old_nodes.empty()) {
            // 若原节点链表为空
            // 删除对应频率到链表的映射
            freq2nodes_.erase(freq);
            if (freq == min_freq_) {
                // 更新最小频率
                min_freq_++;
            }
        }
        // 更新 key 到节点的映射
        key2node_[key] = std::prev(new_nodes.end());
    }

    void removeMinFreqKey() {
        // 找到最小频率对应的节点链表
        auto& nodes = freq2nodes_[min_freq_];
        // 找到要逐出的（访问频率最少且最旧）的节点对应的 key
        const K& key = nodes.front().key;
        // 从链表中逐出节点
        nodes.pop_front();
        if (nodes.empty()) {
            // 若链表为空，删除对应频率到链表的映射
            freq2nodes_.erase(min_freq_);
        }
        // 移除对应 key 到节点的映射
        key2node_.erase(key);
        // 移除对应 key 到频率的映射
        key2freq_.erase(key);
    }

    struct Node {
        K key;
        V value;

        Node() = default;
        Node(const K& k, const V& v) : key(k), value(v) {}
    };
    const size_t capacity_;
    uint32_t min_freq_{0};
    std::mutex mutex_;

    std::unordered_map<K, uint32_t> key2freq_;
    std::unordered_map<uint32_t, std::list<Node>> freq2nodes_;
    std::unordered_map<K, typename std::list<Node>::iterator> key2node_;
};

}  // namespace xsf_simple_cache

#endif  // XSF_LFU_CACHE_H