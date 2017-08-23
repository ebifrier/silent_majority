#ifndef APERY_MOVEPICKER_HPP
#define APERY_MOVEPICKER_HPP

#include "move.hpp"
#include "position.hpp"
#include "search.hpp"

struct HistoryStats {
  static const int Max = 1 << 28;

  int get(Color c, Move m) const { return table[c][m.from()][m.to()]; }
  void clear() { std::memset(table, 0, sizeof(table)); }
  void update(Color c, Move m, int v) {
    Square f = m.from();
    Square t = m.to();
    const int D = 324;
    assert(abs(v) <= D); // Consistency check for below formula
    table[c][f][t] -= table[c][f][t] * abs(v) / D;
    table[c][f][t] += v * 32;
  }

private:
  int table[ColorNum][SquareNum + PieceTypeNum][SquareNum];
};

template<typename T>
struct Stats {
  const T* operator[](Piece pc) const { return table[pc]; }
  T* operator[](Piece pc) { return table[pc]; }
  void clear() { std::memset(table, 0, sizeof(table)); }
  void update(Piece pc, Square to, Move m) { table[pc][to] = m; }
  void update(Piece pc, Square to, int v) {
    const int D = 936;
    assert(abs(int(v)) <= D); // Consistency check for below formula
    table[pc][to] -= table[pc][to] * abs(v) / D;
    table[pc][to] += v * 32;
  }

private:
  T table[PieceNone][SquareNum];
};

typedef Stats<Move> MoveStats;
typedef Stats<int> CounterMoveStats;
typedef Stats<CounterMoveStats> CounterMoveHistoryStats;


class MovePicker {
public:
	MovePicker(const MovePicker&) = delete;
	MovePicker& operator=(const MovePicker&) = delete;

	MovePicker(const Position&, Move, const Depth, const Square);
	MovePicker(const Position&, const Move, Score);
    MovePicker(const Position&, const Move, const Depth, Search::Stack*);
	Move nextMove(bool skipQuiets = false);

private:
	void scoreCaptures();
	template <bool IsDrop> void scoreNonCapturesMinusPro();
	void scoreEvasions();
    ExtMove* begin() { return cur; }
    ExtMove* end() { return endMoves; }

	const Position& pos;
    const Search::Stack* ss;
    Move countermove;
	Depth depth;
	Move ttMove;
	Square recaptureSquare;
	Score threshold;
    int stage;
	ExtMove* cur;
	ExtMove* endMoves;
	ExtMove* endBadCaptures;
	ExtMove moves[MaxLegalMoves];
};

#endif // #ifndef APERY_MOVEPICKER_HPP
