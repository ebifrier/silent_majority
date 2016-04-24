#include "tt.hpp"

TranspositionTable TT; // Our global transposition table

void TranspositionTable::resize(size_t mbSize) { // Mega Byte 指定
	/*// 確保する要素数を取得する。
	size_t newSize = (mbSize << 20) / sizeof(TTCluster);
	newSize = std::max(static_cast<size_t>(1024), newSize); // 最小値は 1024 としておく。
	// 確保する要素数は 2 のべき乗である必要があるので、MSB以外を捨てる。
	const int msbIndex = 63 - firstOneFromMSB(static_cast<u64>(newSize));
	newSize = UINT64_C(1) << msbIndex;

	if (newSize == size)
		// 現在と同じサイズなら何も変更する必要がない。
		return;

	size = newSize;
	delete [] entries;
	entries = new (std::nothrow) TTCluster[newSize];
	if (!entries) {
		std::cerr << "Failed to allocate transposition table: " << mbSize << "MB";
		exit(EXIT_FAILURE);
	}
	clear();
    */
    // 確保する要素数を取得する。
    size_t newSize = (mbSize << 20) / sizeof(Cluster);
    newSize = std::max(static_cast<size_t>(1024), newSize); // 最小値は 1024 としておく。
    // 確保する要素数は 2 のべき乗である必要があるので、MSB以外を捨てる。
    const int msbIndex = 63 - firstOneFromMSB(static_cast<u64>(newSize));
    newSize = UINT64_C(1) << msbIndex;
    size_t newClusterCount = newSize;//size_t(1) << 63 - firstOneFromMSB((mbSize * 1024 * 1024) / sizeof(TTCluster));

    if (newClusterCount == clusterCount)
      return;

    clusterCount = newClusterCount;

    free(mem);
    mem = calloc(clusterCount * sizeof(Cluster) + CacheLineSize - 1, 1);

    if (!mem)
    {
      std::cerr << "Failed to allocate " << mbSize
        << "MB for transposition table." << std::endl;
      exit(EXIT_FAILURE);
    }

    table = (Cluster*)((uintptr_t(mem) + CacheLineSize - 1) & ~(CacheLineSize - 1));
}

void TranspositionTable::clear() {
  std::memset(table, 0, clusterCount * sizeof(Cluster));
}

TTEntry* TranspositionTable::probe(const Key key, bool& found) const {

  TTEntry* const tte = firstEntry(key);
  const u16 key16 = key >> 48;  // Use the high 16 bits as key inside the cluster

  for (unsigned i = 0; i < ClusterSize; ++i)
      if (!tte[i].key16 || tte[i].key16 == key16)
      {
        if ((tte[i].genBound8 & 0xFC) != generation8 && tte[i].key16)
              tte[i].genBound8 = uint8_t(generation8 | tte[i].bound()); // Refresh

          return found = (bool)tte[i].key16, &tte[i];
      }

  // Find an entry to be replaced according to the replacement strategy
  TTEntry* replace = tte;
  for (int i = 1; i < ClusterSize; ++i)
      // Due to our packed storage format for generation and its cyclic
      // nature we add 259 (256 is the modulus plus 3 to keep the lowest
      // two bound bits from affecting the result) to calculate the entry
      // age correctly even after generation8 overflows into the next cycle.
      if (  replace->depth8 - ((259 + generation8 - replace->genBound8) & 0xFC) * 2 * OnePly
          >   tte[i].depth8 - ((259 + generation8 -   tte[i].genBound8) & 0xFC) * 2 * OnePly)
          replace = &tte[i];

  return found = false, replace;
}

/// Returns an approximation of the hashtable occupation during a search. The
/// hash is x permill full, as per UCI protocol.

int TranspositionTable::hashfull() const
{
  int cnt = 0;
  for (int i = 0; i < 1000 / ClusterSize; i++)
  {
      const TTEntry* tte = &table[i].entry[0];
      for (int j = 0; j < ClusterSize; j++)
          if ((tte[j].genBound8 & 0xFC) == generation8)
              cnt++;
  }
  return cnt;
}