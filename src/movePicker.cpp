#include "movePicker.hpp"
#include "generateMoves.hpp"
#include "thread.hpp"

namespace {
const Score LVATable[PieceTypeNum] = {
	Score(0), Score(1), Score(2), Score(3), Score(4), Score(7), Score(8), Score(6), Score(10000),
	Score(5), Score(5), Score(5), Score(5), Score(9), Score(10)
};
inline Score LVA(const PieceType pt) { return LVATable[pt]; }

struct HasPositiveScore { bool operator () (const MoveStack& ms) { return 0 < ms.score; } };

  void insertion_sort(MoveStack* begin, MoveStack* end)
  {
    MoveStack tmp, *p, *q;

    for (p = begin + 1; p < end; ++p)
    {
        tmp = *p;
        for (q = p; q != begin && *(q-1) < tmp; --q)
            *q = *(q-1);
        *q = tmp;
    }
  }

  inline MoveStack* pick_best(MoveStack* begin, MoveStack* end)
  {
      std::swap(*begin, *std::max_element(begin, end));
      return begin;
  }

}// namespace

MovePicker::MovePicker(const Position& p, const Move ttm, const Depth d, Search::Stack* s)
	: pos(p), ss(s), depth(d)
{
	assert(Depth0 < d);

	legalMoves[0].score = INT_MAX; // 番兵のセット
	cur = endMoves = firstMove();
	endBadCaptures = legalMoves + MaxLegalMoves - 1;
    Square prevSq = (ss-1)->currentMove.to();
    countermove = pos.thisThread()->counterMoves[pos.piece(prevSq)][prevSq];

    phase = pos.inCheck() ? EvasionSearch : MainSearch;

	ttMove = (!ttm.isNone() && pos.moveIsPseudoLegal(ttm) ? ttm : Move::moveNone());
	endMoves += (!ttMove.isNone());
}

// 静止探索で呼ばれる。
MovePicker::MovePicker(const Position& p, Move ttm, const Depth d, const Square sq)
	: pos(p), cur(firstMove()), endMoves(firstMove())
{
	assert(d <= Depth0);
	legalMoves[0].score = INT_MAX; // 番兵のセット

	if (pos.inCheck())
		phase = QEvasionSearch;

	// todo: ここで Stockfish は qcheck がある。

	else if (DepthQRecaptures < d)
		phase = QSearch;

	else {
		phase = QRecapture;
		recaptureSquare = sq;
		ttm = Move::moveNone();
	}

	ttMove = (!ttm.isNone() && pos.moveIsPseudoLegal(ttm) ? ttm : Move::moveNone());
	endMoves += !ttMove.isNone();
}

MovePicker::MovePicker(const Position& p, const Move ttm, Score th)
	: pos(p), threshold(th), cur(firstMove()), endMoves(firstMove())
{
	assert(!pos.inCheck());

	legalMoves[0].score = INT_MAX; // 番兵のセット
	phase = ProbCut;
#if 1
    ttMove = !ttm.isNone()
      && pos.moveIsPseudoLegal(ttm)
      && !ttMove.isCapture()
      && pos.see(ttm) > threshold ? ttm : MOVE_NONE;
#else
	ttMove = ((!ttm.isNone() && pos.moveIsPseudoLegal(ttm)) ? ttm : Move::moveNone());

	if (!ttMove.isNone() && (!ttMove.isCapture() || pos.see(ttMove) <= captureThreshold))
		ttMove = Move::moveNone();
#endif
	endMoves += !ttMove.isNone();
}

void MovePicker::scoreCaptures() {
	for (MoveStack* it = cur; it != endMoves; ++it) {
		const Move move = it->move;
		it->score = Position::pieceScore(pos.piece(move.to())) - LVA(move.pieceTypeFrom());
	}
}

template <bool IsDrop> void MovePicker::scoreNonCapturesMinusPro() {
  const HistoryStats& history = pos.thisThread()->history;

  const CounterMoveStats* cm = (ss-1)->counterMoves;
  const CounterMoveStats* fm = (ss-2)->counterMoves;
  const CounterMoveStats* f2 = (ss-4)->counterMoves;

	for (MoveStack* it = cur; it != endMoves; ++it) {
		const Move m = it->move;
#if 1
        it->score = history[pos.moved_piece(m)][m.to()]
          + (cm ? (*cm)[pos.moved_piece(m)][m.to()] : ScoreZero)
          + (fm ? (*fm)[pos.moved_piece(m)][m.to()] : ScoreZero)
          + (f2 ? (*f2)[pos.moved_piece(m)][m.to()] : ScoreZero);
#else
        //Piece pc = colorAndPieceTypeToPiece(pos.turn(), (IsDrop ? move.pieceTypeDropped() : move.pieceTypeFrom()));
		it->score = history.value(IsDrop, pos.moved_piece(move), move.to())
          + (*counterMoveHistory).value(IsDrop, pos.moved_piece(move), move.to());
#endif
	}
}

void MovePicker::scoreEvasions() {
  const HistoryStats& history = pos.thisThread()->history;

	for (MoveStack* it = cur; it != endMoves; ++it) {
		const Move move = it->move;
		const Score seeScore = pos.seeSign(move);
		if (seeScore < 0)
			it->score = seeScore - HistoryStats::Max;
		else if (move.isCaptureOrPromotion()) {
			it->score = pos.capturePieceScore(pos.piece(move.to())) + HistoryStats::Max;
			if (move.isPromotion()) {
				const PieceType pt = pieceToPieceType(pos.piece(move.from()));
				it->score += pos.promotePieceScore(pt);
			}
		}
		else
			//it->score = history.value(move.isDrop(), colorAndPieceTypeToPiece(pos.turn(), move.pieceTypeFromOrDropped()), move.to());
          it->score = history.value(move.isDrop(), pos.moved_piece(move), move.to());
	}
}

void MovePicker::goNextPhase() {
	cur = firstMove(); // legalMoves_[0] は番兵
	++phase;

	switch (phase) {
	case PH_TacticalMoves0: case PH_TacticalMoves1:
		endMoves = generateMoves<CapturePlusPro>(cur, pos);
		scoreCaptures();
		return;

	case PH_Killers:
		cur = killerMoves;
		endMoves = cur + 2;

        killerMoves[0].move = ss->killers[0];
        killerMoves[1].move = ss->killers[1];
        killerMoves[2].move = countermove;
        cur = killerMoves;
        endMoves = cur + 2 + (countermove != killerMoves[0] && countermove != killerMoves[1]);

		return;

	case PH_NonTacticalMoves0:
		endMoves = generateMoves<NonCaptureMinusPro>(cur, pos);
		scoreNonCapturesMinusPro<false>();
		cur = endMoves;
		lastNonCapture = endMoves = generateMoves<Drop>(cur, pos);
		scoreNonCapturesMinusPro<true>();
		cur = firstMove();
		endMoves = std::partition(cur, lastNonCapture, HasPositiveScore());
		// 要素数は10個くらいまでであることが多い。要素数が少ないので、insertionSort() を使用する。
		insertion_sort(cur, endMoves);
		return;

	case PH_NonTacticalMoves1:
		cur = endMoves;
		endMoves = lastNonCapture;
		if (static_cast<Depth>(3 * OnePly) <= depth)
          insertion_sort(cur, endMoves); //std::sort(cur, end, std::greater<MoveStack>());
		return;

	case PH_BadCaptures:
		cur = legalMoves + MaxLegalMoves - 1;
		endMoves = endBadCaptures;
		return;

	case PH_Evasions:
	case PH_QEvasions:
		endMoves = generateMoves<Evasion>(cur, pos);
		if (cur + 1 < endMoves)
			scoreEvasions();
		return;

	case PH_QCaptures0:
		endMoves = generateMoves<CapturePlusPro>(firstMove(), pos);
		scoreCaptures();
		return;

	case PH_QCaptures1:
		endMoves = generateMoves<Recapture>(firstMove(), pos, recaptureSquare);
		scoreCaptures();
		return;

	case EvasionSearch: case QSearch: case QEvasionSearch: case QRecapture: case ProbCut:
		// これが無いと、MainSearch の後に EvasionSearch が始まったりしてしまう。
		phase = PH_Stop;

	case PH_Stop:
		endMoves = cur + 1;
		return;

	default: UNREACHABLE;
	}
}

Move MovePicker::nextMove() {
	MoveStack* ms;
	Move move;
	do {
		// end() に達したら次の phase に移る。
		while (cur == endMoves && phase != PH_Stop)
			goNextPhase();

		switch (phase) {

		case MainSearch: case EvasionSearch: case QSearch: case QEvasionSearch: case ProbCut:
			++cur;
			return ttMove;

		case PH_TacticalMoves0:
			ms = pick_best(cur++, endMoves);
			if (ms->move != ttMove) {
				if (ScoreZero <= pos.see(ms->move))
					return ms->move;

				// 後ろから SEE の点数が高い順に並ぶようにする。
				(endBadCaptures--)->move = ms->move;
			}
			break;

		case PH_Killers:
			move = (cur++)->move;
			if (!move.isNone()
				&& move != ttMove
				&& pos.moveIsPseudoLegal(move, true)
				&& pos.piece(move.to()) == Empty)
			{
				return move;
			}
			break;

		case PH_NonTacticalMoves0:
		case PH_NonTacticalMoves1:
			move = (cur++)->move;
			if (move != ttMove
				&& move != killerMoves[0].move
                && move != killerMoves[1].move
                && move != killerMoves[2].move)
			{
				return move;
			}
			break;

		case PH_BadCaptures:
			return (cur--)->move;

		case PH_Evasions: case PH_QEvasions: case PH_QCaptures0:
			move = pick_best(cur++, endMoves)->move;
			if (move != ttMove)
				return move;
			break;

		case PH_TacticalMoves1:
			ms = pick_best(cur++, endMoves);
			// todo: see が確実に駒打ちじゃないから、内部で駒打ちか判定してるのは少し無駄。
			if (ms->move != ttMove && threshold < pos.see(ms->move))
				return ms->move;
			break;

		case PH_QCaptures1:
			move = pick_best(cur++, endMoves)->move;
			assert(move.to() == recaptureSquare);
			return move;

		case PH_Stop:
			return Move::moveNone();

		default:
			UNREACHABLE;
		}
	} while (true);
}
