#ifndef XSF_ARC_CACHE_H
#define XSF_ARC_CACHE_H

#include <functional>
#include <list>
#include <memory>
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
        if (capacity_ == 0) {
            return;
        }
        if (keys_.size() >= capacity_) {
            pop();
        }
        if (contains(key)) {
            remove(key);
        }
        keys_.push_back(key);
        key2key_[key] = std::prev(keys_.end());
    }

    bool contains(const K& key) const { return key2key_.count(key) != 0; }

    void remove(const K& key) {
        if (!contains(key)) {
            return;
        }
        auto it = key2key_[key];
        keys_.erase(it);
        key2key_.erase(key);
    }

    void increaseCapacity() { capacity_++; }

    void decreaseCapacity() {
        if (capacity_ == 0) {
            return;
        }
        capacity_--;
        if (keys_.size() > capacity_) {
            pop();
        }
    }

   private:
    void pop() {
        const K& old_key = keys_.front();
        key2key_.erase(old_key);
        keys_.pop_front();
    }

    size_t capacity_;
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
        if (capacity_ == 0) {
            return;
        }
        if (contains(key)) {
            // key 已存在，更新
            key2node_[key]->value = value;
            makeRecently(key);
        } else {
            // key 不存在，插入
            if (nodes_.size() >= capacity_) {
                popLeastRecently();
            }
            nodes_.emplace_back(key, value);
            key2node_[key] = std::prev(nodes_.end());
        }
    }

    std::optional<V> get(const K& key) {
        if (!contains(key)) {
            return std::nullopt;
        }
        makeRecently(key);
        return key2node_[key]->value;
    }

    bool contains(const K& key) const { return key2node_.count(key) != 0; }

    uint8_t getCountByKey(const K& key) const {
        return key2node_.at(key)->count;
    }

    void increaseCapacity() {
        capacity_++;
        ghost_list_.increaseCapacity();
    }

    void decreaseCapacity() {
        if (capacity_ == 0) {
            return;
        }
        capacity_--;
        if (nodes_.size() > capacity_) {
            popLeastRecently();
        }
        ghost_list_.decreaseCapacity();
    }

    bool containsInGhostList(const K& key) const {
        return ghost_list_.contains(key);
    }

    void removeInGhostList(const K& key) { ghost_list_.remove(key); }

   private:
    void makeRecently(const K& key) {
        // 增加 count
        key2node_[key]->count++;
        // 移至链表后端
        auto& node_iter = key2node_[key];
        nodes_.splice(nodes_.end(), nodes_, node_iter);
    }

    void popLeastRecently() {
        // 记录将逐出的 key
        const K& old_key = nodes_.front().key;
        // 将 key 移至 ghost_list
        ghost_list_.put(old_key);
        // 移除映射
        key2node_.erase(old_key);
        // 移除链表头部节点
        nodes_.pop_front();
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
        if (capacity_ == 0) {
            return;
        }
        if (contains(key)) {
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
        if (!contains(key)) {
            return std::nullopt;
        }
        increaseFreq(key);
        return key2node_[key]->value;
    }

    bool contains(const K& key) const { return key2node_.count(key) != 0; }

    void increaseCapacity() {
        capacity_++;
        ghost_list_.increaseCapacity();
    }

    void decreaseCapacity() {
        if (capacity_ == 0) {
            return;
        }
        capacity_--;
        if (key2node_.size() > capacity_) {
            removeMinFreqKey();
        }
        ghost_list_.decreaseCapacity();
    }

    bool containsInGhostList(const K& key) const {
        return ghost_list_.contains(key);
    }

    void removeInGhostList(const K& key) { ghost_list_.remove(key); }

   private:
    void increaseFreq(const K& key) {
        // 找到 key 对应的频率
        uint8_t freq = key2freq_[key];
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
        // 将 key 移至 ghost_list
        ghost_list_.put(key);
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
    size_t capacity_;
    uint8_t min_freq_{0};

    std::unordered_map<K, uint8_t, Hash, KeyEqual> key2freq_;
    std::unordered_map<uint8_t, std::list<Node>> freq2nodes_;
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
        if (capacity_ == 0) {
            return;
        }

        bool in_lru = lru_list_.contains(key);
        bool in_lfu = lfu_list_.contains(key);
        if (!in_lru && !in_lfu) {
            // 检查 ghost_list
            adjustCapacityByGhostList(key);
        }
        // 加入或更新 lru_list
        lru_list_.put(key, value);
        if (in_lfu) {
            // 更新 lfu_list
            lfu_list_.put(key, value);
        }
        if (lru_list_.getCountByKey(key) >= k_) {
            // 访问次数大于阈值，加入 lfu_list
            lfu_list_.put(key, value);
        }
    }

    std::optional<V> get(const K& key) override {
        if (capacity_ == 0) {
            return std::nullopt;
        }

        bool in_lru = lru_list_.contains(key);
        bool in_lfu = lfu_list_.contains(key);
        if (!in_lru && !in_lfu) {
            // 检查 ghost_list
            adjustCapacityByGhostList(key);
        }
        if (in_lru) {
            // 从 lru_list 中获取 value
            auto value = lru_list_.get(key).value();
            // 访问次数大于阈值，加入 lfu_list
            if (lru_list_.getCountByKey(key) >= k_) {
                lfu_list_.put(key, value);
            }
            return value;
        } else if (in_lfu) {
            // 从 lfu_list 中获取 value
            auto value = lfu_list_.get(key).value();
            // 加入 lru_list
            lru_list_.put(key, value);
            return value;
        }
        return std::nullopt;
    }

   private:
    void adjustCapacityByGhostList(const K& key) {
        if (lru_list_.containsInGhostList(key)) {
            // key 存在于 lru_list 的 ghost_list 中
            // 从 lru_list 的 ghost_list 中移除 key
            lru_list_.removeInGhostList(key);
            // 增加 lru_list 的容量
            lru_list_.increaseCapacity();
            // 减少 lfu_list 的容量
            lfu_list_.decreaseCapacity();
        } else if (lfu_list_.containsInGhostList(key)) {
            // key 存在于 lfu_list 的 ghost_list 中
            // 从 lfu_list 的 ghost_list 中移除 key
            lfu_list_.removeInGhostList(key);
            // 增加 lfu_list 的容量
            lfu_list_.increaseCapacity();
            // 减少 lru_list 的容量
            lru_list_.decreaseCapacity();
        }
    }

    const size_t capacity_;
    const uint8_t k_{DEFAULT_K};
    XSFArcLruList<K, V, Hash, KeyEqual> lru_list_;
    XSFArcLfuList<K, V, Hash, KeyEqual> lfu_list_;
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