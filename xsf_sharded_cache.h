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
        if (isZeroCapacity()) {
            return;
        }
        initializeShards(creator);
    }

    void put(const K& key, const V& value) override {
        if (isZeroCapacity()) {
            return;
        }
        auto* shard = getShardForKey(key);
        shard->put(key, value);
    }

    std::optional<V> get(const K& key) override {
        if (isZeroCapacity()) {
            return std::nullopt;
        }
        auto* shard = getShardForKey(key);
        return shard->get(key);
    }

   private:
    bool isZeroCapacity() const { return capacity_ == 0; }

    void initializeShards(XSFCacheCreator<K, V>& creator) {
        auto capacities = calculateShardCapacities();
        createShards(creator, capacities);
    }

    std::vector<size_t> calculateShardCapacities() const {
        size_t base_capacity = capacity_ / shard_num_;
        size_t remainder = capacity_ % shard_num_;

        std::vector<size_t> capacities;
        capacities.reserve(shard_num_);

        for (size_t i = 0; i < shard_num_; ++i) {
            size_t shard_capacity =
                calculateIndividualShardCapacity(i, base_capacity, remainder);
            capacities.push_back(shard_capacity);
        }

        return capacities;
    }

    size_t calculateIndividualShardCapacity(size_t shard_index,
                                            size_t base_capacity,
                                            size_t remainder) const {
        size_t capacity = base_capacity;
        if (shard_index < remainder) {
            capacity++;
        }
        return ensureMinimumCapacity(capacity);
    }

    size_t ensureMinimumCapacity(size_t capacity) const {
        return std::max(capacity, size_t(1));
    }

    void createShards(XSFCacheCreator<K, V>& creator,
                      const std::vector<size_t>& capacities) {
        shards_.reserve(shard_num_);
        for (size_t capacity : capacities) {
            shards_.emplace_back(creator.create(capacity));
        }
    }

    XSFCache<K, V>* getShardForKey(const K& key) {
        size_t shard_index = calculateShardIndex(key);
        return shards_[shard_index].get();
    }

    size_t calculateShardIndex(const K& key) const {
        return hash_(key) % shard_num_;
    }

    const size_t capacity_;
    const size_t shard_num_;
    Hash hash_;
    std::vector<std::unique_ptr<XSFCache<K, V>>> shards_;
};

}  // namespace xsf_simple_cache

#endif  // XSF_SHARDED_CACHE_H