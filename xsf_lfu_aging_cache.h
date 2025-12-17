#ifndef XSF_LFU_AGING_CACHE_H
#define XSF_LFU_AGING_CACHE_H

#include <list>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "xsf_cache.h"

namespace xsf_simple_cache {

template <typename K, typename V, typename Hash = std::hash<K>,
          typename KeyEqual = std::equal_to<K>>
class XSFLfuAgingCache : public XSFCache<K, V> {
   public:
    explicit XSFLfuAgingCache(size_t capacity, uint32_t aging_threshold = 10,
                              Hash hash = Hash{},
                              KeyEqual key_equal = KeyEqual{})
        : capacity_(capacity),
          aging_threshold_(aging_threshold),
          key2freq_(0, hash, key_equal),
          key2node_(0, hash, key_equal) {}

    void put(const K& key, const V& value) override {
        if (capacity_ == 0) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
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
            // 更新平均频率并进行衰减检查
            updateAvgFreq(1);
        }
    }

    std::optional<V> get(const K& key) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (key2node_.count(key) == 0) {
            return std::nullopt;
        }
        increaseFreq(key);
        return key2node_[key]->value;
    }

   private:
    void updateAvgFreq(uint32_t freq) {
        if (key2freq_.size() == 0) {
            avg_freq_ = 0;
            return;
        }
        avg_freq_ =
            (avg_freq_ * (key2freq_.size() - 1) + freq) / key2freq_.size();
        if (avg_freq_ < aging_threshold_) {
            return;
        }
        // 平均频率超过阈值，进行衰减
        uint32_t decay = aging_threshold_ / 2;
        // 衰减所有 key 的频率，更新最小频率，计算新的平均频率
        uint32_t new_avg_freq = 0;
        for (auto& [k, f] : key2freq_) {
            if (f > decay) {
                f -= decay;
                min_freq_ = std::min(min_freq_, f);
            } else {
                f = 1;
                min_freq_ = 1;
            }
            new_avg_freq += f;
        }
        avg_freq_ = new_avg_freq / key2freq_.size();
        // 重新构建频率到节点、key到节点的映射
        std::vector<std::pair<K, V>> items;
        for (auto& [k, it] : key2node_) {
            items.emplace_back(k, it->value);
        }
        freq2nodes_.clear();
        key2node_.clear();
        for (const auto& [k, v] : items) {
            uint32_t f = key2freq_[k];
            freq2nodes_[f].emplace_back(k, v);
            key2node_[k] = std::prev(freq2nodes_[f].end());
        }
    }

    void increaseFreq(const K& key) {
        // 找到 key 对应的频率
        uint32_t freq = key2freq_[key];
        // 更新 key 到频率的映射
        key2freq_[key]++;

        // 在新频率对应的节点链表中插入新节点
        auto& new_nodes = freq2nodes_[freq + 1];
        V value = key2node_[key]->value;
        new_nodes.emplace_back(key, value);
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
        // 更新平均频率并进行衰减检查
        updateAvgFreq(freq + 1);
    }

    void removeMinFreqKey() {
        // 找到最小频率对应的节点链表
        auto& nodes = freq2nodes_[min_freq_];
        // 找到要逐出的（访问频率最少且最旧）的节点对应的 key
        const K& key = nodes.front().key;
        // 移除对应 key 到节点的映射
        key2node_.erase(key);
        // 移除对应 key 到频率的映射
        key2freq_.erase(key);
        // 从链表中逐出节点
        nodes.pop_front();
        if (nodes.empty()) {
            // 若链表为空，删除对应频率到链表的映射
            freq2nodes_.erase(min_freq_);
        }
    }

    struct Node {
        K key;
        V value;

        Node() = default;
        Node(const K& k, const V& v) : key(k), value(v) {}
    };
    const size_t capacity_;
    const uint32_t aging_threshold_{10};
    uint32_t min_freq_{0};
    uint32_t avg_freq_{0};
    std::mutex mutex_;

    std::unordered_map<K, uint32_t, Hash, KeyEqual> key2freq_;
    std::unordered_map<uint32_t, std::list<Node>> freq2nodes_;
    std::unordered_map<K, typename std::list<Node>::iterator, Hash, KeyEqual>
        key2node_;
};

}  // namespace xsf_simple_cache

#endif  // XSF_LFU_AGING_CACHE_H