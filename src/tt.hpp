#ifndef APERY_TT_HPP
#define APERY_TT_HPP

#include "common.hpp"
#include "move.hpp"

enum Depth {
	OnePly                 =  1,
	Depth0                 =  0 * OnePly,
	Depth1                 =  1 * OnePly,
	DepthQChecks           =  0 * OnePly,
	DepthQNoChecks         = -1 * OnePly,
	DepthQRecaptures       = -5 * OnePly,
	DepthNone              = -6 * OnePly,
    DepthMax = MaxPly * OnePly
};
OverloadEnumOperators(Depth);

static_assert(!(OnePly & (OnePly - 1)), "OnePly is not a power of 2");

struct TTEntry {

	Move  move() const       { return static_cast<Move>(move16); }
    Score score() const      { return static_cast<Score>(score16); }
	Score evalScore() const  { return static_cast<Score>(evalScore16); }
    Depth depth() const      { return static_cast<Depth>(depth8 * int(OnePly)); }
    Bound bound() const      { return static_cast<Bound>(genBound8 & 0x3); }

	void save(Key k, Score s, Bound b, Depth d,  Move m, Score es, u8 g)
	{
        assert(d / OnePly * OnePly == d);

        if (m || (k >> 48) != key16)
            move16 = static_cast<u16>(m.value());

        // Don't overwrite more valuable entries
        if ((k >> 48) != key16
          || d / OnePly > depth8 - 4
       /* || g != (genBound8 & 0xFC) // Matching non-zero keys are already refreshed by probe() */
          || b == BoundExact)
        {
            key16       = static_cast<u16>(k >> 48);
            score16     = static_cast<s16>(s);
            evalScore16 = static_cast<s16>(es);
            genBound8   = static_cast<u8>(g | b);
            depth8      = static_cast<s8>(d / OnePly);
        }
	}

private:
    friend class TranspositionTable;

	u16 key16;
	u16 move16;
	s16 score16;
	s16 evalScore16;
    u8 genBound8;
    s8 depth8;
};

class TranspositionTable {

  //static const int CacheLineSize = 64;
  static const int ClusterSize = 3;

  struct Cluster {
    TTEntry entry[ClusterSize];
    char padding[2]; // Align to the cache line size
  };

public:
    ~TranspositionTable() { free(mem); }
    void newSearch() { generation8 += 4; }
    u8 generation() const { return generation8; }
    TTEntry* probe(const Key key, bool& found) const;
    int hashfull() const;
	void resize(size_t mbSize); // Mega Byte 指定
	void clear();

    // The lowest order bits of the key are used to get the index of the cluster
    TTEntry* firstEntry(const Key key) const {
      return &table[(size_t)key & (clusterCount - 1)].entry[0];
    }

private:
    size_t clusterCount;
    Cluster* table;
    void* mem;
    u8 generation8; // Size must be not bigger than TTEntry::genBound8
};

extern TranspositionTable TT;

#endif // #ifndef APERY_TT_HPP
