#ifndef APERY_MOVEPICKER_HPP
#define APERY_MOVEPICKER_HPP

#include "move.hpp"
#include "position.hpp"
#include "search.hpp"

enum GenerateMovePhase {
	MainSearch, PH_TacticalMoves0, PH_Killers, PH_NonTacticalMoves0, PH_NonTacticalMoves1, PH_BadCaptures,
	EvasionSearch, PH_Evasions,
	QSearch, PH_QCaptures0,
	QEvasionSearch, PH_QEvasions,
	ProbCut, PH_TacticalMoves1,
	QRecapture, PH_QCaptures1,
	PH_Stop
};
OverloadEnumOperators(GenerateMovePhase); // ++phase_ の為。


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


class MovePicker {
public:
	MovePicker(const Position&, Move, const Depth, const Square);
	MovePicker(const Position&, const Move, Score);
    MovePicker(const Position&, const Move, const Depth, Search::Stack*);
	Move nextMove();

private:
	void scoreCaptures();
	template <bool IsDrop> void scoreNonCapturesMinusPro();
	void scoreEvasions();
	void goNextPhase();

	MoveStack* firstMove() { return &legalMoves[1]; } // [0] は番兵

	const Position& pos;
    const Search::Stack* ss;
    Move countermove;
	Depth depth;
	Move ttMove; // transposition table move
	MoveStack killerMoves[3];
	Square recaptureSquare;
	Score threshold; // int で良いのか？
	GenerateMovePhase phase;
	MoveStack* cur;
	MoveStack* endMoves;
	MoveStack* lastNonCapture;
	MoveStack* endBadCaptures;
	// std::array にした方が良さそう。
	MoveStack legalMoves[MaxLegalMoves];
};

#endif // #ifndef APERY_MOVEPICKER_HPP
