#ifndef APERY_MOVEPICKER_HPP
#define APERY_MOVEPICKER_HPP

#include "move.hpp"
#include "position.hpp"
#include "search.hpp"

template<typename T>
struct Stats {
  const T* operator[](Piece pc) const { return table[pc]; }
  T* operator[](Piece pc) { return table[pc]; }
  void clear() { std::memset(table, 0, sizeof(table)); }
  void update(Piece pc, Square to, Move m) { table[pc][to] = m; }

  void update(Piece pc, Square to, Score v) {

    if (abs(int(v)) >= 324)
      return;

    table[pc][to] -= table[pc][to] * abs(int(v)) / 936;
    table[pc][to] += int(v) * 32;
  }

private:
  T table[PieceNone][SquareNum];
};

typedef Stats<Move> MoveStats;
typedef Stats<Score> CounterMoveStats;
typedef Stats<CounterMoveStats> CounterMoveHistoryStats;


struct HistoryStats {
  static const Score Max = Score(1 << 28);

  Score get(Color c, Move m) const { return table[c][m.from()][m.to()]; }
  void clear() { std::memset(table, 0, sizeof(table)); }

  void update(Color c, Move m, Score s)
  {
    if (abs(int(s)) >= 324)
      return;

    Square f = m.from();
    Square t = m.to();

    table[c][f][t] -= table[c][f][t] * abs(int(s)) / 324;
    table[c][f][t] += int(s) * 32;
  }

private:
  Score table[ColorNum][SquareNum + PieceTypeNum][SquareNum];
};


class MovePicker {
public:
	MovePicker(const MovePicker&) = delete;
	MovePicker& operator=(const MovePicker&) = delete;

	MovePicker(const Position&, Move, const Depth, const Square);
	MovePicker(const Position&, const Move, Score);
    MovePicker(const Position&, const Move, const Depth, Search::Stack*);
	Move nextMove();

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
