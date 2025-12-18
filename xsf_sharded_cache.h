#ifndef XSF_SHARDED_CACHE_H
#define XSF_SHARDED_CACHE_H

#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "xsf_cache.h"

namespace xsf_simple_cache {

template <typename K, typename V, typename Hash = std::hash<K>>
class XSFShardedCache : public XSFCache<K, V> {
   public:
    XSFShardedCache(size_t capacity, XSFCacheCreator<K, V>& creator,
                    size_t shard_num = std::thread::hardware_concurrency(),
                    Hash hash = Hash())
        : capacity_(capacity),
          shard_num_(std::max(size_t(1), shard_num)),
          hash_(std::move(hash)) {
        if (capacity_ == 0) {
            return;
        }
        // 计算每个分片的基础容量和余数
        size_t base_capacity = capacity_ / shard_num_;
        size_t remainder = capacity_ % shard_num_;

        // 创建分片
        shards_.reserve(shard_num_);
        for (size_t i = 0; i < shard_num_; ++i) {
            // 前remainder个分片容量为base_capacity+1
            // 其余分片容量为base_capacity
            size_t shard_capacity = base_capacity;
            if (i < remainder) {
                shard_capacity++;
            }
            // 确保每个分片至少容量为1
            shard_capacity = std::max(shard_capacity, size_t(1));
            shards_.emplace_back(creator.create(shard_capacity));
        }
    }

    void put(const K& key, const V& value) override {
        if (capacity_ == 0) {
            return;
        }
        size_t shard_index = hash_(key) % shard_num_;
        shards_[shard_index]->put(key, value);
    }

    std::optional<V> get(const K& key) override {
        if (capacity_ == 0) {
            return std::nullopt;
        }
        size_t shard_index = hash_(key) % shard_num_;
        return shards_[shard_index]->get(key);
    }

   private:
    const size_t capacity_;
    const size_t shard_num_;
    Hash hash_;
    std::vector<std::unique_ptr<XSFCache<K, V>>> shards_;
};

}  // namespace xsf_simple_cache

#endif  // XSF_SHARDED_CACHE_H