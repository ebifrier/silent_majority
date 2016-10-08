﻿#ifndef APERY_MOVEPICKER_HPP
#define APERY_MOVEPICKER_HPP

#include "move.hpp"
#include "position.hpp"
#include "search.hpp"

template<typename T, bool CM = false>
struct Stats {

  static const Score Max = Score(1 << 28);

  const T* operator[](Piece pc) const { return table[pc]; }
  T* operator[](Piece pc) { return table[pc]; }
  void clear() { std::memset(table, 0, sizeof(table)); }

  Score value(const bool isDrop, const Piece pc, const Square to) const {
    assert(0 < pc && pc < PieceNone);
    assert(isInSquare(to));
    return table[pc][to];
  }

  void update(Piece pc, Square to, Move m) { table[pc][to] = m; }

  void update(Piece pc, Square to, Score v) {

    if (abs(int(v)) >= 324)
      return;

    table[pc][to] -= table[pc][to] * abs(int(v)) / (CM ? 936 : 324);
    table[pc][to] += int(v) * 32;
  }

private:
  T table[PieceNone][SquareNum];
};

typedef Stats<Move> MoveStats;
typedef Stats<Score, false> HistoryStats;
typedef Stats<Score, true> CounterMoveStats;
typedef Stats<CounterMoveStats> CounterMoveHistoryStats;

#ifdef FROMTO
struct FromToStats {

  Score get(Color c, Move m) const { return table[c][m.isDrop() ? SquareNum : m.from()][m.to()]; }
  void clear() { std::memset(table, 0, sizeof(table)); }

  void update(Color c, Move m, Score s)
  {
    if (abs(int(s)) >= 324)
      return;

    Square f = m.isDrop() ? SquareNum : m.from();
    Square t = m.to();

    table[c][f][t] -= table[c][f][t] * abs(int(s)) / 324;
    table[c][f][t] += int(s) * 32;
  }

private:
  Score table[ColorNum][SquareNum + 1][SquareNum];
};
#endif


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
    MoveStack* begin() { return cur; }
    MoveStack* end() { return endMoves; }

	const Position& pos;
    const Search::Stack* ss;
    Move countermove;
	Depth depth;
	Move ttMove;
	Square recaptureSquare;
	Score threshold;
    int stage;
	MoveStack* cur;
	MoveStack* endMoves;
	MoveStack* endBadCaptures;
	MoveStack moves[MaxLegalMoves];
};

#endif // #ifndef APERY_MOVEPICKER_HPP
