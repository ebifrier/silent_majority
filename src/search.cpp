﻿#include "search.hpp"
#include "position.hpp"
#include "usi.hpp"
#include "evaluate.hpp"
#include "movePicker.hpp"
#include "tt.hpp"
#include "generateMoves.hpp"
#include "thread.hpp"
#include "timeManager.hpp"
#include "book.hpp"

namespace Search {

	SignalsType Signals;
	LimitsType Limits;
	StateStackPtr SetUpStates;

    std::vector<Move> SearchMoves;

#if defined LEARN
	STATIC Score alpha;
	STATIC Score beta;
#endif
#if defined INANIWA_SHIFT
	InaniwaFlag inaniwaFlag;
#endif
};

using namespace Search;

namespace {
    enum NodeType { NonPV, PV };

    // Sizes and phases of the skip-blocks, used for distributing search depths across the threads
    const int skipSize[] = { 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4 };
    const int skipPhase[] = { 0, 1, 0, 1, 2, 3, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 6, 7 };

	const Score Tempo = Score(20); // Must be visible to search
    const int razorMargin[] = { 0, 570, 603, 554 };
    inline Score futilityMargin(const Depth d) { return Score(150 * d / OnePly);}

    int FutilityMoveCounts[2][16]; // [improving][depth]
	int Reductions[2][2][64][64]; // [pv][improving][depth][moveNumber]

    // Threshold used for countermoves based pruning.
    const int CounterMovePruneThreshold = 0;

	template <bool PVNode> inline Depth reduction(const bool i, const Depth depth, const int mn) {
		return Reductions[PVNode][i][std::min(int(depth / OnePly), 63)][std::min(mn, 63)] * OnePly;
	}

	// History and stats update bonus, based on depth
	int statBonus(Depth depth) {
		int d = depth / OnePly;
		return d > 17 ? 0 : Score(d * d + 2 * d - 2);
	}

	struct Skill {
		Skill(int l) : level(l) {}
		bool enabled() const { return level < 20; }
		bool time_to_pick(Depth depth) const { return depth / OnePly == 1 + level; }
		Move best_move(size_t multiPV) { return best ? best : pick_best(multiPV); }
		Move pick_best(size_t multiPV);

		int level;
		Move best = MOVE_NONE;
	};

    struct EasyMoveManager {

      void clear() {
        stableCnt = 0;
        expectedPosKey = 0;
        pv[0] = pv[1] = pv[2] = MOVE_NONE;
      }

      Move get(Key key) const {
        return expectedPosKey == key ? pv[2] : MOVE_NONE;
      }

      void update(Position& pos, const std::vector<Move>& newPv) {

        assert(newPv.size() >= 3);

        // Keep track of how many times in a row 3rd ply remains stable
        stableCnt = (newPv[2] == pv[2]) ? stableCnt + 1 : 0;

        if (!std::equal(newPv.begin(), newPv.begin() + 3, pv))
        {
          std::copy(newPv.begin(), newPv.begin() + 3, pv);

          StateInfo st[2];
          pos.doMove(newPv[0], st[0]);
          pos.doMove(newPv[1], st[1]);
          expectedPosKey = pos.getKey();
          pos.undoMove(newPv[1]);
          pos.undoMove(newPv[0]);
        }
      }

      int stableCnt;
      Key expectedPosKey;
      Move pv[3];
    };

    EasyMoveManager EasyMove;
	Score DrawScore[ColorNum];

    template <NodeType NT>
    Score search(Position& pos, Search::Stack* ss, Score alpha, Score beta, const Depth depth, const bool cutNode, bool skipEarlyPruning);

    template <NodeType NT, bool INCHECK>
    Score qsearch(Position& pos, Search::Stack* ss, Score alpha, Score beta, const Depth depth = Depth0);

#if defined INANIWA_SHIFT
    void detectInaniwa(const Position& pos);
#endif

    void checkTime();

#if 1
	Move Skill::pick_best(size_t multiPV) {

		const RootMoves& rootMoves = Threads.main()->rootMoves;
		static PRNG rng(now());

		Score topScore = rootMoves[0].score;
		int delta = std::min(topScore - rootMoves[multiPV - 1].score, PawnScore);
		int weakness = 120 - 2 * level;
		int maxScore = -ScoreInfinite;

		for (size_t i = 0; i < multiPV; ++i)
		{
			int push = (weakness * int(topScore - rootMoves[i].score)
						+ delta * (rng.rand<unsigned>() % weakness)) / 128;

			if (rootMoves[i].score + push > maxScore)
			{
				maxScore = rootMoves[i].score + push;
				best = rootMoves[i].pv[0];
			}
		}

		return best;
	}
#endif
	Score scoreToTT(const Score s, const Ply ply) {
		assert(s != ScoreNone);

		return (ScoreMateInMaxPly <= s ? s + ply
				: s <= ScoreMatedInMaxPly ? s - ply
				: s);
	}

	Score scoreFromTT(const Score s, const Ply ply) {
		return (s == ScoreNone ? ScoreNone
				: ScoreMateInMaxPly <= s ? s - ply
				: s <= ScoreMatedInMaxPly ? s + ply
				: s);
	}

	void updatePV(Move* pv, Move move, Move* childPv) {

		for (*pv++ = move; childPv && *childPv != MOVE_NONE; )
			*pv++ = *childPv++;
		*pv = MOVE_NONE;
	}

	void updateCMStats(Stack* ss, Piece pc, Square s, int bonus) {

        for (int i : {1, 2, 4})
            if ((ss-i)->currentMove.isOK())
                (ss-i)->counterMoves->update(pc, s, bonus);
	}

	void updateStats(const Position& pos, Stack* ss, Move move,
					  Move* quiets, int quietsCnt, int bonus) {

		if (ss->killers[0] != move)
		{
			ss->killers[1] = ss->killers[0];
			ss->killers[0] = move;
		}

		Color c = pos.turn();
		Thread* thisThread = pos.thisThread();
		thisThread->history.update(c, move, bonus);
		updateCMStats(ss, pos.movedPiece(move), move.to(), bonus);

		if ((ss-1)->currentMove.isOK())
		{
			Square prevSq = (ss-1)->currentMove.to();
			thisThread->counterMoves.update(pos.piece(prevSq), prevSq, move);
		}

		// Decrease all the other played quiet moves
		for (int i = 0; i < quietsCnt; ++i)
		{
			thisThread->history.update(c, quiets[i], -bonus);
			updateCMStats(ss, pos.movedPiece(quiets[i]), quiets[i].to(), -bonus);
		}
	}

	std::string scoreToUSI(const Score score) {
		std::stringstream ss;

		if (abs(score) < ScoreMateInMaxPly)
			// cp は centi pawn の略
			ss << "cp " << score * 100 / PawnScore;
		else
			// mate の後には、何手で詰むかを表示する。
			ss << "mate " << (0 < score ? ScoreMate0Ply - score : -ScoreMate0Ply - score);

		return ss.str();
	}

#if defined BISHOP_IN_DANGER
	BishopInDangerFlag detectBishopInDanger(const Position& pos) {
		if (pos.gamePly() <= 60) {
			const Color them = oppositeColor(pos.turn());
			if (pos.hand(pos.turn()).exists<HBishop>()
				&& pos.bbOf(Silver, them).isSet(inverseIfWhite(them, SQ27))
				&& (pos.bbOf(King  , them).isSet(inverseIfWhite(them, SQ48))
					|| pos.bbOf(King  , them).isSet(inverseIfWhite(them, SQ47))
					|| pos.bbOf(King  , them).isSet(inverseIfWhite(them, SQ59)))
				&& pos.bbOf(Pawn  , them).isSet(inverseIfWhite(them, SQ37))
				&& pos.piece(inverseIfWhite(them, SQ28)) == Empty
				&& pos.piece(inverseIfWhite(them, SQ38)) == Empty
				&& pos.piece(inverseIfWhite(them, SQ39)) == Empty)
			{
				return (pos.turn() == Black ? BlackBishopInDangerIn28 : WhiteBishopInDangerIn28);
			}
			else if (pos.hand(pos.turn()).exists<HBishop>()
					 && pos.hand(them).exists<HBishop>()
					 && pos.piece(inverseIfWhite(them, SQ78)) == Empty
					 && pos.piece(inverseIfWhite(them, SQ79)) == Empty
					 && pos.piece(inverseIfWhite(them, SQ68)) == Empty
					 && pos.piece(inverseIfWhite(them, SQ69)) == Empty
					 && pos.piece(inverseIfWhite(them, SQ98)) == Empty
					 && (pieceToPieceType(pos.piece(inverseIfWhite(them, SQ77))) == Silver
						 || pieceToPieceType(pos.piece(inverseIfWhite(them, SQ88))) == Silver)
					 && (pieceToPieceType(pos.piece(inverseIfWhite(them, SQ77))) == Knight
						 || pieceToPieceType(pos.piece(inverseIfWhite(them, SQ89))) == Knight)
					 && ((pieceToPieceType(pos.piece(inverseIfWhite(them, SQ58))) == Gold
						  && pieceToPieceType(pos.piece(inverseIfWhite(them, SQ59))) == King)
						 || pieceToPieceType(pos.piece(inverseIfWhite(them, SQ59))) == Gold))
			{
				return (pos.turn() == Black ? BlackBishopInDangerIn78 : WhiteBishopInDangerIn78);
			}
			else if (pos.hand(pos.turn()).exists<HBishop>()
					 && pos.hand(them).exists<HBishop>()
					 && pos.piece(inverseIfWhite(them, SQ38)) == Empty
					 && pos.piece(inverseIfWhite(them, SQ18)) == Empty
					 && pieceToPieceType(pos.piece(inverseIfWhite(them, SQ28))) == Silver
					 && (pieceToPieceType(pos.piece(inverseIfWhite(them, SQ58))) == King
						 || pieceToPieceType(pos.piece(inverseIfWhite(them, SQ57))) == King
						 || pieceToPieceType(pos.piece(inverseIfWhite(them, SQ58))) == Gold
						 || pieceToPieceType(pos.piece(inverseIfWhite(them, SQ57))) == Gold)
					 && (pieceToPieceType(pos.piece(inverseIfWhite(them, SQ59))) == King
						 || pieceToPieceType(pos.piece(inverseIfWhite(them, SQ59))) == Gold))
			{
				return (pos.turn() == Black ? BlackBishopInDangerIn38 : WhiteBishopInDangerIn38);
			}
		}
		return NotBishopInDanger;
	}
#endif

std::string pvInfoToUSI(Position& pos, const Depth depth, const Score alpha, const Score beta) {
	std::stringstream ss;
    const int t = Time.elapsed() + 1;
    const RootMoves& rootMoves = pos.thisThread()->rootMoves;
    size_t pvIdx = pos.thisThread()->pvIdx;
    size_t multiPV = std::min((size_t)Options["MultiPV"], rootMoves.size());
    uint64_t nodesSearched = Threads.nodes_searched();

    for (size_t i = multiPV - 1; 0 <= static_cast<int>(i); --i) {
		const bool update = (i <= pvIdx);

		if (depth == OnePly && !update)
			continue;

		const Depth d = (update ? depth : depth - OnePly);
		const Score s = (update ? rootMoves[i].score : rootMoves[i].previousScore);

        if (ss.rdbuf()->in_avail()) // Not at first line
            ss << "\n";

		ss << "info depth " << d / OnePly
		   << " seldepth " << pos.thisThread()->maxPly
           << " multipv " << i + 1
		   << " score " << scoreToUSI(s);

        if (i == pvIdx)
           ss << (s >= beta ? " lowerbound" : s <= alpha ? " upperbound" : "");

        ss << " nodes " << nodesSearched
           << " nps " << nodesSearched * 1000 / t;

        if (t > 1000) // Earlier makes little sense
            ss << " hashfull " << TT.hashfull();

		ss << " time " << t
		   << " pv";

		for (Move m : rootMoves[i].pv)
			ss << " " << m.toUSI();

		ss << std::endl;
	}
	return ss.str();
}
} // namespace

void Search::init() {

  for (int imp = 0; imp <= 1; ++imp)
      for (int d = 1; d < 64; ++d)
          for (int mc = 1; mc < 64; ++mc)
          {
              double r = log(d) * log(mc) / 1.95;

              Reductions[NonPV][imp][d][mc] = int(std::round(r));
              Reductions[PV][imp][d][mc] = std::max(Reductions[NonPV][imp][d][mc] - 1, 0);

              // Increase reduction for non-PV nodes when eval is not improving
              if (!imp && Reductions[NonPV][imp][d][mc] >= 2)
                Reductions[NonPV][imp][d][mc]++;
          }

  for (int d = 0; d < 16; ++d)
  {
      FutilityMoveCounts[0][d] = int(2.4 + 0.74 * pow(d, 1.78));
      FutilityMoveCounts[1][d] = int(5.0 + 1.00 * pow(d, 2.00));
  }
}

void Search::clear() {

	TT.clear();

	for (Thread* th : Threads)
	{
		th->history.clear();
		th->counterMoves.clear();
		th->counterMoveHistory.clear();
		th->resetCalls = true;
        CounterMoveStats& cm = th->counterMoveHistory[Empty][0];
        int* t = &cm[Empty][0];
        std::fill(t, t + sizeof(cm), CounterMovePruneThreshold - 1);
	}

	Threads.main()->previousScore = ScoreInfinite;
}

// 入玉勝ちかどうかを判定
bool nyugyoku(const Position& pos) {
	// CSA ルールでは、一 から 六 の条件を全て満たすとき、入玉勝ち宣言が出来る。

	// 一 宣言側の手番である。

	// この関数を呼び出すのは自分の手番のみとする。ponder では呼び出さない。

	const Color us = pos.turn();
	// 敵陣のマスク
	const Bitboard opponentsField = (us == Black ? inFrontMask<Black, Rank4>() : inFrontMask<White, Rank6>());

	// 二 宣言側の玉が敵陣三段目以内に入っている。
	if (!pos.bbOf(King, us).andIsAny(opponentsField))
		return false;

	// 三 宣言側が、大駒5点小駒1点で計算して
	//     先手の場合28点以上の持点がある。
	//     後手の場合27点以上の持点がある。
	//     点数の対象となるのは、宣言側の持駒と敵陣三段目以内に存在する玉を除く宣言側の駒のみである。
	const Bitboard bigBB = pos.bbOf(Rook, Dragon, Bishop, Horse) & opponentsField & pos.bbOf(us);
	const Bitboard smallBB = (pos.bbOf(Pawn, Lance, Knight, Silver) | pos.goldsBB()) & opponentsField & pos.bbOf(us);
	const Hand hand = pos.hand(us);
	const int val = (bigBB.popCount() + hand.numOf<HRook>() + hand.numOf<HBishop>()) * 5
		+ smallBB.popCount()
		+ hand.numOf<HPawn>() + hand.numOf<HLance>() + hand.numOf<HKnight>()
		+ hand.numOf<HSilver>() + hand.numOf<HGold>();
#if defined LAW_24
	if (val < 31)
		return false;
#else
	if (val < (us == Black ? 28 : 27))
		return false;
#endif

	// 四 宣言側の敵陣三段目以内の駒は、玉を除いて10枚以上存在する。

	// 玉は敵陣にいるので、自駒が敵陣に11枚以上あればよい。
	if ((pos.bbOf(us) & opponentsField).popCount() < 11)
		return false;

	// 五 宣言側の玉に王手がかかっていない。
	if (pos.inCheck())
		return false;

	// 六 宣言側の持ち時間が残っている。

	// 持ち時間が無ければ既に負けなので、何もチェックしない。

	return true;
}

void MainThread::search() {
	static Book book;
    Position& pos = rootPos;
    Color us = pos.turn();
    bool isbook = false;
	Time.init(Limits, us, pos.gamePly());
	std::uniform_int_distribution<int> dist(Options["Min_Book_Ply"], Options["Max_Book_Ply"]);
	const Ply book_ply = dist(g_randomTimeSeed);
	bool nyugyokuWin = false;

	int contempt = Options["Contempt"] * PawnScore / 100; // From centipawns
	DrawScore[us] = ScoreDraw - Score(contempt);
	DrawScore[~us] = ScoreDraw + Score(contempt);

	// 指し手が無ければ負け
	if (rootMoves.empty()) {
		rootMoves.push_back(RootMove(Move::moveNone()));
		SYNCCOUT << "info depth 0 score "
			<< scoreToUSI(-ScoreMate0Ply)
			<< SYNCENDL;

		goto finalize;
	}

#if defined LEARN
#else
	if (nyugyoku(pos)) {
		nyugyokuWin = true;
		goto finalize;
	}
#endif

#if defined LEARN
	threads[0]->searching = true;
#else

	SYNCCOUT << "info string book_ply " << book_ply << SYNCENDL;
	if (Options["OwnBook"] && pos.gamePly() <= book_ply) {
		const std::tuple<Move, Score> bookMoveScore = book.probe(pos, Options["Book_File"], Options["Best_Book_Move"]);
		if (std::get<0>(bookMoveScore) && std::find(rootMoves.begin(),
															  rootMoves.end(),
															  std::get<0>(bookMoveScore)) != rootMoves.end())
		{
			std::swap(rootMoves[0], *std::find(rootMoves.begin(),
											   rootMoves.end(),
											   std::get<0>(bookMoveScore)));
			SYNCCOUT << "info"
					 << " score " << scoreToUSI(std::get<1>(bookMoveScore))
					 << " pv " << std::get<0>(bookMoveScore).toUSI()
					 << SYNCENDL;

            isbook = true;
			goto finalize;
		}
	}
#if defined BISHOP_IN_DANGER
	{
		auto deleteFunc = [](const std::string& str) {
			auto it = std::find_if(std::begin(rootMoves), std::end(rootMoves), [&str](const RootMove& rm) {
					return rm.pv_[0].toCSA() == str;
				});
			if (it != std::end(rootMoves))
				rootMoves.erase(it);
		};
		switch (detectBishopInDanger(pos)) {
		case NotBishopInDanger: break;
		case BlackBishopInDangerIn28: deleteFunc("0082KA"); break;
		case WhiteBishopInDangerIn28: deleteFunc("0028KA"); break;
		case BlackBishopInDangerIn78: deleteFunc("0032KA"); break;
		case WhiteBishopInDangerIn78: deleteFunc("0078KA"); break;
		case BlackBishopInDangerIn38: deleteFunc("0072KA"); break;
		case WhiteBishopInDangerIn38: deleteFunc("0038KA"); break;
		default: UNREACHABLE;
		}
	}
#endif

#if defined INANIWA_SHIFT
	detectInaniwa(pos);
#endif

    for (Thread* th : Threads)
    {
      th->maxPly = 0;
      th->rootDepth = Depth0;
      if (th != this)
      {
        th->rootPos = Position(rootPos, th);
        th->rootMoves = rootMoves;
        th->start_searching();
      }
    }
#endif
    Thread::search(); // Let's start searching!

#if defined LEARN
#else

finalize:
	if (Limits.npmsec)
		Time.availableNodes += Limits.inc[us] - Threads.nodes_searched();

	if (!Signals.stop && (Limits.ponder || Limits.infinite)) {
		Signals.stopOnPonderhit = true;
		wait(Signals.stop);
	}

    // Wait until all threads have finished
	for (Thread* th : Threads)
		if (th != this)
			th->wait_for_search_finished();

    // Check if there are threads with a better score than main thread
    Thread* bestThread = this;
	if (!this->easyMovePlayed
		&& !isbook
		&&  Options["MultiPV"] == 1
		&& !Limits.depth
		&& !Skill(Options["Skill_Level"]).enabled()
		&& rootMoves[0].pv[0] != MOVE_NONE)
	{
		for (Thread* th : Threads) {
			const Depth depthDiff = th->completedDepth - bestThread->completedDepth;
			const Score scoreDiff = th->rootMoves[0].score - bestThread->rootMoves[0].score;

			if (scoreDiff > 0 && depthDiff >= 0)
				bestThread = th;
		}
	}

    previousScore = bestThread->rootMoves[0].score;

    if (bestThread != this)
        SYNCCOUT << pvInfoToUSI(bestThread->rootPos, bestThread->completedDepth, -ScoreInfinite, ScoreInfinite) << SYNCENDL;

#ifdef RESIGN
    if (!isbook && previousScore < -Options["Resign"] 
		&& !Skill(Options["Skill_Level"]).enabled()) // 
        SYNCCOUT << "bestmove resign" << SYNCENDL;
#endif
    if (nyugyokuWin)
        SYNCCOUT << "bestmove win" << SYNCENDL;
    else if (!bestThread->rootMoves[0].pv[0])
        SYNCCOUT << "bestmove resign" << SYNCENDL;
    else {
        SYNCCOUT << "bestmove " << bestThread->rootMoves[0].pv[0].toUSI();
        if (bestThread->rootMoves[0].pv.size() > 1 || bestThread->rootMoves[0].extractPonderFromTT(pos))
            std::cout << " ponder " << bestThread->rootMoves[0].pv[1].toUSI();
        
        std::cout << SYNCENDL;
    }
#endif
}

void Thread::search() {

    Stack stack[MaxPly+7], *ss = stack+4; // To allow referencing (ss-5) and (ss+2)
    Score bestScore, alpha, beta, delta;
    Move easyMove = MOVE_NONE;
    MainThread* mainThread = (this == Threads.main() ? Threads.main() : nullptr);
    int lastInfoTime = -1; // 将棋所のコンソールが詰まる問題への対処用
	int pv_interval = Options["PvInterval"]; // PVの出力間隔[ms]

	std::memset(ss-4, 0, 7 * sizeof(Stack));
    for (int i = 4; i > 0; i--)
        (ss-i)->counterMoves = &this->counterMoveHistory[Empty][0]; // Use as sentinel


    bestScore = delta = alpha = -ScoreInfinite;
    beta = ScoreInfinite;
    completedDepth = Depth0;

	if (mainThread)
	{
		easyMove = EasyMove.get(rootPos.getKey());
		EasyMove.clear();
		mainThread->easyMovePlayed = mainThread->failedLow = false;
		mainThread->bestMoveChanges = 0;
		TT.newSearch();
	}

#if defined LEARN
	// 高速化の為に浅い探索は反復深化しないようにする。学習時は浅い探索をひたすら繰り返す為。
	depth = std::max<Ply>(0, limits.depth - 1);
#else

#endif

	size_t multiPV = Options["MultiPV"];
	Skill skill(Options["Skill_Level"]);

	if (skill.enabled())
		multiPV = std::max(multiPV, (size_t)4);

	multiPV = std::min(multiPV, rootMoves.size());

	// 反復深化で探索を行う。
	while ( (rootDepth = rootDepth + OnePly) < DepthMax
		   && !Signals.stop
		   && (!Limits.depth || Threads.main()->rootDepth / OnePly <= Limits.depth)) {

        // Distribute search depths across the threads
        if (idx) {
            int i = (idx - 1) % 20;
            if (((rootDepth / OnePly + rootPos.gamePly() + skipPhase[i]) / skipSize[i]) % 2)
                continue;
        }

		if (mainThread)
			mainThread->bestMoveChanges *= 0.505, mainThread->failedLow = false;

		// 前回の iteration の結果を全てコピー
		for (RootMove& rm : rootMoves)
			rm.previousScore = rm.score;

		// Multi PV loop
		for (pvIdx = 0; pvIdx < multiPV && !Signals.stop; ++pvIdx) {
#if defined LEARN
			alpha = this->alpha;
			beta = this->beta;
#else
			// aspiration search
			// alpha, beta をある程度絞ることで、探索効率を上げる。
			if (rootDepth >= 5 * OnePly) {
				delta = static_cast<Score>(18);
				alpha = std::max(rootMoves[pvIdx].previousScore - delta, -ScoreInfinite);
				beta  = std::min(rootMoves[pvIdx].previousScore + delta,  ScoreInfinite);
			}
#endif

			// aspiration search の window 幅を、初めは小さい値にして探索し、
			// fail high/low になったなら、今度は window 幅を広げて、再探索を行う。
			while (true) {
				// 探索を行う。
				(ss-1)->staticEvalRaw.p[0][0] = ss->staticEvalRaw.p[0][0] = ScoreNotEvaluated;
				bestScore = ::search<PV>(rootPos, ss, alpha, beta, rootDepth, false, false);
				// 先頭が最善手になるようにソート
				std::stable_sort(rootMoves.begin() + pvIdx, rootMoves.end());

#if 0
                // 詰みを発見したら即指す。
                if (!Limits.ponder && ScoreMateInMaxPly <= abs(bestScore) && abs(bestScore) < ScoreInfinite) {
                    SYNCCOUT << pvInfoToUSI(rootPos, rootDepth, alpha, beta) << SYNCENDL;
                    Signals.stop = true;
                }
#endif

#if defined LEARN
				break;
#endif

				if (Signals.stop)
					break;

#ifdef USE_extractPVFromTT
				if (mainThread
					&& multiPV == 1
					&& (bestScore <= alpha || bestScore >= beta)
					&& 3000 < Time.elapsed()
					// 将棋所のコンソールが詰まるのを防ぐ。
					&& (60000 < Time.elapsed() || lastInfoTime + 3000 < Time.elapsed()))
				{
					lastInfoTime = Time.elapsed();
					SYNCCOUT << pvInfoToUSI(rootPos, rootDepth, alpha, beta) << SYNCENDL;
				}
#endif
				// fail high/low のとき、aspiration window を広げる。
				if (bestScore <= alpha) {
					// 勝ち(負け)だと判定したら、最大の幅で探索を試してみる。
					beta = (alpha + beta) / 2;
					alpha = std::max(bestScore - delta, -ScoreInfinite);

					if (mainThread)
					{
						mainThread->failedLow = true;
						Signals.stopOnPonderhit = false;
					}
				}
				else if (bestScore >= beta) {
					alpha = (alpha + beta) / 2;
					beta = std::min(bestScore + delta, ScoreInfinite);
				}
				else
					break;

				delta += delta / 4 + 5;

				assert(-ScoreInfinite <= alpha && beta <= ScoreInfinite);
			}

			std::stable_sort(rootMoves.begin(), rootMoves.begin() + pvIdx + 1);

			if (!mainThread)
				continue;

			if ((Signals.stop || pvIdx + 1 == multiPV || 3000 < Time.elapsed())
			    // 将棋所のコンソールが詰まるのを防ぐ。
			    && (rootDepth < 4 || lastInfoTime + pv_interval < Time.elapsed()))
			{
				lastInfoTime = Time.elapsed();
				SYNCCOUT << pvInfoToUSI(rootPos, rootDepth, alpha, beta) << SYNCENDL;
			}
		}

		if (!Signals.stop)
			completedDepth = rootDepth;

		if (!mainThread)
			continue;

		if (skill.enabled() && skill.time_to_pick(rootDepth))
			skill.pick_best(multiPV);

#if 0
		// Have we found a "mate in x"?
		if (Limits.mate
			&& bestScore >= ScoreMateInMaxPly
			&& ScoreMate0Ply - bestScore <= 2 * Limits.mate)
			Signals.stop = true;

#endif

		if (Limits.useTimeManagement()) {
			if (!Signals.stop && !Signals.stopOnPonderhit) {

				const int F[] = { mainThread->failedLow,
				  bestScore - mainThread->previousScore };

				int improvingFactor = std::max(229, std::min(715, 357 + 119 * F[0] - 6 * F[1]));
				double unstablePvFactor = 1 + mainThread->bestMoveChanges;

				bool doEasyMove = rootMoves[0].pv[0] == easyMove
					&& mainThread->bestMoveChanges < 0.03
					&& Time.elapsed() > Time.optimum() * 5 / 44;

				if (rootMoves.size() == 1
					|| Time.elapsed() > Time.optimum() * unstablePvFactor * improvingFactor / 628
					|| (mainThread->easyMovePlayed = doEasyMove))
				{
					if (Limits.ponder)
						Signals.stopOnPonderhit = true;
					else
						Signals.stop = true;
				}
			}

			if (rootMoves[0].pv.size() >= 3)
				EasyMove.update(rootPos, rootMoves[0].pv);
			else
				EasyMove.clear();
		}
	}

	if (!mainThread)
		return;

	if (EasyMove.stableCnt < 6 || mainThread->easyMovePlayed)
		EasyMove.clear();

	if (skill.enabled())
		std::swap(rootMoves[0], *std::find(rootMoves.begin(),
										   rootMoves.end(), skill.best_move(multiPV)));

	//SYNCCOUT << pvInfoToUSI(rootPos, rootDepth-1, alpha, beta) << SYNCENDL;
}

#if defined INANIWA_SHIFT
// 稲庭判定
void Searcher::detectInaniwa(const Position& pos) {
	if (inaniwaFlag == NotInaniwa && 20 <= pos.gamePly()) {
		const Rank Trank3 = (pos.turn() == Black ? Rank3 : Rank7); // not constant
		const Bitboard mask = rankMask(Trank3) & ~fileMask<File9>() & ~fileMask<File1>();
		if ((pos.bbOf(Pawn, oppositeColor(pos.turn())) & mask) == mask) {
			inaniwaFlag = (pos.turn() == Black ? InaniwaIsWhite : InaniwaIsBlack);
			tt.clear();
		}
	}
}
#endif

namespace {
template <NodeType NT>
Score search(Position& pos, Stack* ss, Score alpha, Score beta, const Depth depth, const bool cutNode, bool skipEarlyPruning) {

    const bool PvNode = NT == PV;
    const bool rootNode = PvNode && (ss - 1)->ply == 0;

	assert(-ScoreInfinite <= alpha && alpha < beta && beta <= ScoreInfinite);
	assert(PvNode || (alpha == beta - 1));
	assert(Depth0 < depth && depth < DepthMax);
    assert(!(PvNode && cutNode));
    assert(depth / OnePly * OnePly == depth);

    Move pv[MaxPly+1], quietsSearched[64];
	StateInfo st;
	TTEntry* tte;
	Key posKey;
	Move ttMove, move, excludedMove, bestMove;
	Depth extension, newDepth;
	Score bestScore, score, ttScore, eval;
    bool ttHit, inCheck, givesCheck, singularExtensionNode, improving;
	bool captureOrPawnPromotion, doFullDepthSearch, moveCountPruning, skipQuiets;
    Piece movedPiece;
	int moveCount, quietCount;
	Square prevSq;

	// step1
	// initialize node
    Thread* thisThread = pos.thisThread();
    inCheck = pos.inCheck();
    moveCount = quietCount = ss->moveCount = 0;
	ss->history = 0;
    bestScore = -ScoreInfinite;
    ss->ply = (ss-1)->ply + 1;

    if (thisThread->resetCalls.load(std::memory_order_relaxed))
    {
        thisThread->resetCalls = false;
		// At low node count increase the checking rate to about 0.1% of nodes
		// otherwise use a default value.
		thisThread->callsCnt = Limits.nodes ? std::min(4096, int(Limits.nodes / 1024))
			                                : 4096;
    }
    if (--thisThread->callsCnt <= 0)
    {
        for (Thread* th : Threads)
            th->resetCalls = true;

        checkTime();
    }

	if (PvNode && thisThread->maxPly < ss->ply)
		thisThread->maxPly = ss->ply;

	if (!rootNode) {
		// step2
		// stop と最大探索深さのチェック
		switch (pos.isDraw(16)) {
		case NotRepetition      : if (!Signals.stop.load(std::memory_order_relaxed) && ss->ply <= MaxPly) { break; }
		case RepetitionDraw     : return DrawScore[pos.turn()];
		case RepetitionWin      : return mateIn(ss->ply);
		case RepetitionLose     : return matedIn(ss->ply);
		case RepetitionSuperior : if (ss->ply != 2) { return ScoreMateInMaxPly; } break;
		case RepetitionInferior : if (ss->ply != 2) { return ScoreMatedInMaxPly; } break;
		default                 : UNREACHABLE;
		}

		// step3
		// mate distance pruning
		alpha = std::max(matedIn(ss->ply), alpha);
		beta = std::min(mateIn(ss->ply+1), beta);
		if (beta <= alpha)
			return alpha;
	}

    ss->currentMove = (ss+1)->excludedMove = bestMove = Move::moveNone();
    ss->counterMoves = &thisThread->counterMoveHistory[Empty][0];
	(ss+2)->killers[0] = (ss+2)->killers[1] = Move::moveNone();
	prevSq = (ss-1)->currentMove.to();

	// step4
	// trans position table lookup
	excludedMove = ss->excludedMove;
#if 1
	posKey = pos.getKey() ^ Key(excludedMove.value() << 1);
#else
	posKey = (!excludedMove ? pos.getKey() : pos.getExclusionKey());
#endif
	tte = TT.probe(posKey, ttHit);
    ttScore = (ttHit ? scoreFromTT(tte->score(), ss->ply) : ScoreNone);
	ttMove = (rootNode ? thisThread->rootMoves[thisThread->pvIdx].pv[0]
			  : ttHit ? move16toMove(tte->move(), pos)
			  : Move::moveNone());

	if (!PvNode
		&& ttHit
		&& tte->depth() >= depth
		&& ttScore != ScoreNone // アクセス競合が起きたときのみ、ここに引っかかる。
		&& (ttScore >= beta ? (tte->bound() & BoundLower)
			                : (tte->bound() & BoundUpper)))
	{
		if (ttMove)
		{
			if (ttScore >= beta)
			{
				if (!ttMove.isCaptureOrPawnPromotion())
					updateStats(pos, ss, ttMove, nullptr, 0, statBonus(depth));

				// Extra penalty for a quiet TT move in previous ply when it gets refuted
				if ((ss-1)->moveCount == 1
					&& !(ss-1)->currentMove.isCaptureOrPawnPromotion())
					updateCMStats(ss-1, pos.piece(prevSq), prevSq, -statBonus(depth + OnePly));
			}
			// Penalty for a quiet ttMove that fails low
			else if (!ttMove.isCaptureOrPawnPromotion())
			{
				int penalty = -statBonus(depth);
				thisThread->history.update(pos.turn(), ttMove, penalty);
				updateCMStats(ss, pos.movedPiece(ttMove), ttMove.to(), penalty);
			}
		}
		return ttScore;
	}

#if 1
	if (pos.gamePly() > 200 && nyugyoku(pos)) {
        bestScore = mateIn(ss->ply);
        tte->save(posKey, scoreToTT(bestScore, ss->ply), BoundExact, DepthMax - OnePly,
                  Move::moveNone(), ScoreNone, TT.generation());
        return bestScore;
	}
#endif
#if 1
	if (!rootNode
		&& !inCheck)
	{
		if (move = pos.mateMoveIn1Ply()) {
			bestScore = mateIn(ss->ply);
			tte->save(posKey, scoreToTT(bestScore, ss->ply), BoundExact, DepthMax - OnePly,
					  move, bestScore, TT.generation());
			return bestScore;
		}
	}
#endif

	// step5
	// evaluate the position statically
	eval = ss->staticEval = evaluate(pos, ss); // Bonanza の差分評価の為、evaluate() を常に呼ぶ。
	if (inCheck) {
		eval = ss->staticEval = ScoreNone;
		goto moves_loop;
	}
	else if (ttHit) {
		if (ttScore != ScoreNone
			&& (tte->bound() & (eval < ttScore ? BoundLower : BoundUpper)))
		{
			eval = ttScore;
		}
	}
	else {
#if 1
        if ((ss-1)->currentMove == MOVE_NULL)
            eval = ss->staticEval = -(ss-1)->staticEval + 2 * Tempo;
#endif
		tte->save(posKey, ScoreNone, BoundNone, DepthNone, Move::moveNone(), 
				  ss->staticEval, TT.generation());
	}

    if (skipEarlyPruning)
        goto moves_loop;

	// step6
	// razoring
	if (!PvNode
		&& depth < 4 * OnePly
		&& eval + razorMargin[depth / OnePly] <= alpha)
	{
		if (depth <= OnePly)
			return qsearch<NonPV, false>(pos, ss, alpha, alpha+1);

		const Score ralpha = alpha - razorMargin[depth / OnePly];
		const Score s = qsearch<NonPV, false>(pos, ss, ralpha, ralpha+1);
		if (s <= ralpha)
			return s;
	}

	// step7
	// Futility pruning: child node (skipped when in check)
	if (!rootNode
		&& depth < 7 * OnePly
		&& eval - futilityMargin(depth) >= beta
		&& eval < ScoreKnownWin)
	{
		return eval;
	}

	// step8
	// null move
	if (!PvNode
		&& eval >= beta
        && (ss->staticEval >= beta - 35 * (depth / OnePly - 6) || depth >= 13 * OnePly))
	{
        assert(eval - beta >= 0);
        
        Depth R = ((823 + 67 * depth / OnePly) / 256 + std::min(int(eval - beta) / PawnScore, 3)) * OnePly;

        ss->currentMove = Move::moveNull();
        ss->counterMoves = &thisThread->counterMoveHistory[Empty][0];

		pos.doNullMove<true>(st);
		(ss+1)->staticEvalRaw = (ss)->staticEvalRaw; // 評価値の差分評価の為。
		Score nullScore = (depth-R < OnePly ? -qsearch<NonPV, false>(pos, ss+1, -beta, -beta+1)
		                                    : -search<NonPV>(pos, ss+1, -beta, -beta+1, depth-R, !cutNode, true));
		pos.doNullMove<false>(st);

		if (nullScore >= beta) {
			if (nullScore >= ScoreMateInMaxPly)
				nullScore = beta;

			if (depth < 12 * OnePly && abs(beta) < ScoreKnownWin)
				return nullScore;

			assert(Depth0 < depth - R);
			const Score s = (depth-R < OnePly ? qsearch<NonPV, false>(pos, ss, beta-1, beta)
                                              : search<NonPV>(pos, ss, beta-1, beta, depth-R, false, true));

			if (s >= beta)
				return nullScore;
		}
	}

	// step9
	// probcut
	if (!PvNode
		&& depth >= 5 * OnePly
		&& abs(beta) < ScoreMateInMaxPly)
	{
		const Score rbeta = std::min(beta + 200, ScoreInfinite);
		const Depth rdepth = depth - 4 * OnePly;
        const Score threshold = rbeta - ss->staticEval;

		assert(rdepth >= OnePly);
		assert((ss-1)->currentMove.isOK());

		MovePicker mp(pos, ttMove, threshold);
		const CheckInfo ci(pos);
		while ((move = mp.nextMove()) != MOVE_NONE) {
			if (pos.pseudoLegalMoveIsLegal<false, false>(move, ci.pinned)) {
				ss->currentMove = move;
                ss->counterMoves = &thisThread->counterMoveHistory[pos.movedPiece(move)][move.to()];
				pos.doMove(move, st, ci, pos.moveGivesCheck(move, ci));
				(ss+1)->staticEvalRaw.p[0][0] = ScoreNotEvaluated;
				score = -search<NonPV>(pos, ss+1, -rbeta, -rbeta+1, rdepth, !cutNode, false);
				pos.undoMove(move);
				if (score >= rbeta)
					return score;
			}
		}
	}

	// step10
	// internal iterative deepening
	if (depth >= 6 * OnePly
		&& !ttMove
		&& (PvNode || (ss->staticEval + 256 >= beta)))
	{
        Depth d = (3 * depth / (4 * OnePly) - 2) * OnePly;
		search<NT>(pos, ss, alpha, beta, d, cutNode, true);

		tte = TT.probe(posKey, ttHit);
		ttMove = (ttHit ? move16toMove(tte->move(), pos) : Move::moveNone());
	}

moves_loop:
    const CounterMoveStats& cmh = *(ss-1)->counterMoves;
    const CounterMoveStats& fmh = *(ss-2)->counterMoves;
    const CounterMoveStats& fm2 = *(ss-4)->counterMoves;

	MovePicker mp(pos, ttMove, depth, ss);
	const CheckInfo ci(pos);
	score = bestScore;
    improving =   ss->staticEval >= (ss-2)->staticEval
             /*|| ss->staticEval == ScoreNone // Already implicit in the previous condition */
               ||(ss-2)->staticEval == ScoreNone;

	singularExtensionNode = (   !rootNode
							 &&  depth >= 8 * OnePly
							 &&  ttMove != MOVE_NONE
							 &&  ttScore != ScoreNone
							 && !excludedMove
							 && (tte->bound() & BoundLower)
							 &&  tte->depth() >= depth - 3 * OnePly);
    skipQuiets = false;

	// step11
	// Loop through moves
	while ((move = mp.nextMove(skipQuiets)) != MOVE_NONE) {
		if (move == excludedMove)
			continue;

		if (rootNode
			&& std::find(thisThread->rootMoves.begin() + thisThread->pvIdx,
              thisThread->rootMoves.end(),
						 move) == thisThread->rootMoves.end())
		{
			continue;
		}

		ss->moveCount = ++moveCount;

#if 0
		if (rootNode && thisThread == Threads.main() && 3000 < Timer.elapsed()) {
				SYNCCOUT << "info depth " << depth
						 << " currmove " << move.toUSI()
						 << " currmovenumber " << moveCount + pvIdx << SYNCENDL;
			}
#endif

        if (PvNode)
            (ss+1)->pv = nullptr;

		extension = Depth0;
		captureOrPawnPromotion = move.isCaptureOrPawnPromotion();
        movedPiece = pos.movedPiece(move);
		givesCheck = pos.moveGivesCheck(move, ci);

		moveCountPruning = (depth < 16 * OnePly
							&& moveCount >= FutilityMoveCounts[improving][depth / OnePly]);

		// step12
		// Singular and Gives Check Extensions
        if (singularExtensionNode
            && move == ttMove
            && pos.pseudoLegalMoveIsLegal<false, false>(move, ci.pinned))
        {
            const Score rBeta = std::max(ttScore - 2 * depth / OnePly, -ScoreMate0Ply);
            Depth d = (depth / (2 * OnePly)) * OnePly;
            ss->excludedMove = move;
            score = search<NonPV>(pos, ss, rBeta-1, rBeta, d, cutNode, true);
            ss->excludedMove = Move::moveNone();

            if (score < rBeta) {
                extension = OnePly;
            }
        } 
        else if (givesCheck
                 && !moveCountPruning
                 &&  pos.seeGe(move, ScoreZero))
            extension = OnePly;

		newDepth = depth - OnePly + extension;

		// step13
		// futility pruning
		if (!rootNode
			&& ScoreMatedInMaxPly < bestScore)
		{
			if (!captureOrPawnPromotion
				&& !givesCheck)
			{
				// move count based pruning
                if (moveCountPruning) {
#if 0
                    skipQuiets = true; // 要検討
#endif
                    continue;
                }

				int lmrDepth = std::max(newDepth - reduction<PvNode>(improving, depth, moveCount), Depth0) / OnePly;

				// Countermoves based pruning
				if (lmrDepth < 3
					&& (cmh[movedPiece][move.to()] < CounterMovePruneThreshold)
					&& (fmh[movedPiece][move.to()] < CounterMovePruneThreshold))
					continue;

				// score based pruning
				if (lmrDepth < 7
					&& !inCheck
					&& ss->staticEval + 256 + 200 * lmrDepth <= alpha)
					continue;

				if (lmrDepth < 8
					&& !pos.seeGe(move, Score(-35 * lmrDepth * lmrDepth)))
					continue;
			}
			else if (depth < 7 * OnePly
					 && !extension
					 && !pos.seeGe(move, Score(-CapturePawnScore * int(depth / OnePly))))
				continue;
		}


		// RootNode, SPNode はすでに合法手であることを確認済み。
		if (!rootNode && !pos.pseudoLegalMoveIsLegal<false, false>(move, ci.pinned)) {
			ss->moveCount = --moveCount;
			continue;
		}

		ss->currentMove = move;
        ss->counterMoves = &thisThread->counterMoveHistory[movedPiece][move.to()];

		// step14
		pos.doMove(move, st, ci, givesCheck);
		(ss+1)->staticEvalRaw.p[0][0] = ScoreNotEvaluated;

		// step15
		// LMR
		if (depth >= 3 * OnePly
			&& moveCount > 1
			&& (!captureOrPawnPromotion || moveCountPruning))
		{
			Depth r = reduction<PvNode>(improving, depth, moveCount);

			if (captureOrPawnPromotion)
				r -= r ? OnePly : Depth0;
			else {

				// Increase reduction for cut nodes and moves with a bad history
				if (cutNode)
					r += 2 * OnePly;

#if 0
				// Decrease reduction for moves that escape a capture
				else if (!move.isDrop()//type_of(move) == NORMAL
						 //&& pieceToPieceType(pos.piece(move.to())) != Pawn //type_of(pos.piece(move.to())) != Pawn
						 && move.isPromotion()
						 && !pos.seeGe(makeMove(move.pieceTypeFrom(), move.to(), move.from()), ScoreZero)
					r -= 2 * OnePly;
#endif

				ss->history = thisThread->history.get(~pos.turn(), move)
					+ cmh[movedPiece][move.to()]
					+ fmh[movedPiece][move.to()]
					+ fm2[movedPiece][move.to()]
					- 4000; // Correction factor

				// Decrease/increase reduction by comparing opponent's stat score
				if (ss->history > 0 && (ss-1)->history < 0)
					 r -= OnePly;
				
				else if (ss->history < 0 && (ss-1)->history > 0)
					 r += OnePly;

				// Decrease/increase reduction for moves with a good/bad history
				r = std::max(Depth0, (r / OnePly - ss->history / 20000) * OnePly);
			}
			const Depth d = std::max(newDepth - r, OnePly);

			// PVS
			score = -search<NonPV>(pos, ss+1, -(alpha+1), -alpha, d, true, false);

			doFullDepthSearch = (alpha < score && d != newDepth);
		}
		else
			doFullDepthSearch = (!PvNode || moveCount > 1);

		// step16
		// full depth search
		// PVS
		if (doFullDepthSearch) {
			score = (newDepth < OnePly ?
			  (givesCheck ? -qsearch<NonPV,  true>(pos, ss+1, -(alpha+1), -alpha)
					      : -qsearch<NonPV, false>(pos, ss+1, -(alpha+1), -alpha))
					      : - search<NonPV>(pos, ss+1, -(alpha+1), -alpha, newDepth, !cutNode, false));
		}

		// 通常の探索
		if (PvNode && (moveCount == 1 || (alpha < score && (rootNode || score < beta)))) {

            (ss+1)->pv = pv;
            (ss+1)->pv[0] = MOVE_NONE;

			score = (newDepth < OnePly ?
			  (givesCheck ? -qsearch<PV,  true>(pos, ss+1, -beta, -alpha)
					      : -qsearch<PV, false>(pos, ss+1, -beta, -alpha))
					      : - search<PV>(pos, ss+1, -beta, -alpha, newDepth, false, false));
		}

		// step17
		pos.undoMove(move);

		assert(-ScoreInfinite < score && score < ScoreInfinite);

		// step18
		if (Signals.stop.load(std::memory_order_relaxed))
			return ScoreZero;

		if (rootNode) {
            RootMove& rm = *std::find(thisThread->rootMoves.begin(), thisThread->rootMoves.end(), move);

			if (moveCount == 1 || alpha < score) {
				// PV move or new best move
				rm.score = score;
#ifdef USE_extractPVFromTT
				rm.extractPVFromTT(pos);
#else
                rm.pv.resize(1);

                assert((ss+1)->pv);

                for (Move* m = (ss+1)->pv; *m != MOVE_NONE; ++m)
                    rm.pv.push_back(*m);
#endif
				if (moveCount > 1 && thisThread == Threads.main())
					++static_cast<MainThread*>(thisThread)->bestMoveChanges;
			}
			else
				rm.score = -ScoreInfinite;
		}

		if (score > bestScore) {
			bestScore = score;

			if (score > alpha) {

				bestMove = move;

				if (PvNode && !rootNode) // Update pv even in fail-high case
					updatePV(ss->pv, move, (ss+1)->pv);

				if (PvNode && score < beta)
					alpha = score;
				else {
					// fail high
					assert(score >= beta);
					break;
				}
			}
		}

        if (!captureOrPawnPromotion && move != bestMove && quietCount < 64)
            quietsSearched[quietCount++] = move;
    }

	// step20
	if (moveCount == 0)
		bestScore = excludedMove ? alpha : matedIn(ss->ply);

    else if (bestMove)
    {
        // Quiet best move: update killers, history and countermoves
        if (!bestMove.isCaptureOrPawnPromotion())
            updateStats(pos, ss, bestMove, quietsSearched, quietCount, statBonus(depth));

        // Extra penalty for a quiet TT move in previous ply when it gets refuted
        if ((ss-1)->moveCount == 1 
			&& !(ss-1)->currentMove.isCaptureOrPawnPromotion())
            updateCMStats(ss-1, pos.piece(prevSq), prevSq, -statBonus(depth + OnePly));
    }
	else if (depth >= 3 * OnePly
			 && !(ss-1)->currentMove.isCaptureOrPawnPromotion()
			 && (ss-1)->currentMove.isOK())
		updateCMStats(ss-1, pos.piece(prevSq), prevSq, statBonus(depth));

    if (!excludedMove)
        tte->save(posKey, scoreToTT(bestScore, ss->ply),
                  bestScore >= beta ? BoundLower :
                  PvNode && bestMove ? BoundExact : BoundUpper,
                  depth, bestMove, ss->staticEval, TT.generation());

	assert(-ScoreInfinite < bestScore && bestScore < ScoreInfinite);

	return bestScore;
}

template <NodeType NT, bool INCHECK>
Score qsearch(Position& pos, Stack* ss, Score alpha, Score beta, const Depth depth) {
	const bool PVNode = (NT == PV);
    assert(NT == PV || NT == NonPV);
	assert(INCHECK == pos.inCheck());
	assert(-ScoreInfinite <= alpha && alpha < beta && beta <= ScoreInfinite);
	assert(PVNode || (alpha == beta - 1));
	assert(depth <= Depth0);
    assert(depth / OnePly * OnePly == depth);

    Move pv[MaxPly+1];
	StateInfo st;
	TTEntry* tte;
	Key posKey;
	Move ttMove, move, bestMove;
	Score bestScore, score, ttScore, futilityScore, futilityBase, oldAlpha;
    bool ttHit, givesCheck, evasionPrunable;
	Depth ttDepth;

    if (PVNode) {
        oldAlpha = alpha;
        (ss+1)->pv =pv;
        ss->pv[0] = MOVE_NONE;
    }

	ss->currentMove = bestMove = Move::moveNone();
	ss->ply = (ss-1)->ply + 1;

	if (ss->ply >= MaxPly)
		return DrawScore[pos.turn()];

	assert(0 <= ss->ply && ss->ply < MaxPly);

	ttDepth = ((INCHECK || DepthQChecks <= depth) ? DepthQChecks : DepthQNoChecks);

	posKey = pos.getKey();
	tte = TT.probe(posKey, ttHit);
	ttMove = (ttHit ? move16toMove(tte->move(), pos) : Move::moveNone());
	ttScore = (ttHit ? scoreFromTT(tte->score(), ss->ply) : ScoreNone);

	if (!PVNode
        && ttHit
		&& tte->depth() >= ttDepth
		&& ttScore != ScoreNone // アクセス競合が起きたときのみ、ここに引っかかる。
		&& (ttScore >= beta ? (tte->bound() & BoundLower)
			                : (tte->bound() & BoundUpper)))
	{
		return ttScore;
	}

	if (INCHECK) {
		ss->staticEval = ScoreNone;
		bestScore = futilityBase = -ScoreInfinite;
	}
	else {
		if (move = pos.mateMoveIn1Ply())
			return mateIn(ss->ply);

		if (ttHit) {
			if ((ss->staticEval = bestScore = tte->evalScore()) == ScoreNone)
				ss->staticEval = bestScore = evaluate(pos, ss);

			if (ttScore != ScoreNone)
				if (tte->bound() & (ttScore > bestScore ? BoundLower : BoundUpper))
					bestScore = ttScore;
		}
		else
			ss->staticEval = bestScore = 
            (((ss-1)->currentMove != MOVE_NULL) ? evaluate(pos, ss) 
                                                : -(ss-1)->staticEval + 2 * Tempo);

        // Stand pat
		if (bestScore >= beta) {
			if (!ttHit)
				tte->save(posKey, scoreToTT(bestScore, ss->ply), BoundLower,
						  DepthNone, Move::moveNone(), ss->staticEval, TT.generation());

			return bestScore;
		}

		if (PVNode && bestScore > alpha)
			alpha = bestScore;

		futilityBase = bestScore + 128; // todo: 128 より大きくて良いと思う。
	}

	evaluate(pos, ss);

	MovePicker mp(pos, ttMove, depth, (ss-1)->currentMove.to());
	const CheckInfo ci(pos);

    while ((move = mp.nextMove()) != MOVE_NONE)
	{
		assert(pos.isOK());

		givesCheck = pos.moveGivesCheck(move, ci);

		// futility pruning
		if (!INCHECK // 駒打ちは王手回避のみなので、ここで弾かれる。
			&& !givesCheck
			&& futilityBase > -ScoreKnownWin)
		{
			futilityScore = futilityBase + Position::capturePieceScore(pos.piece(move.to()));
			if (move.isPromotion())
				futilityScore += Position::promotePieceScore(move.pieceTypeFrom());

			if (futilityScore <= alpha) {
				bestScore = std::max(bestScore, futilityScore);
				continue;
			}

			if (futilityBase <= alpha && !pos.seeGe(move, ScoreZero + 1)) {
				bestScore = std::max(bestScore, futilityBase);
				continue;
			}
		}

		evasionPrunable = (INCHECK
						   && bestScore > ScoreMatedInMaxPly
						   && !move.isCaptureOrPawnPromotion());

		if ((!INCHECK || evasionPrunable)
			&& (!move.isPromotion() || move.pieceTypeFrom() != Pawn)
			&& !pos.seeGe(move, ScoreZero))
		{
			continue;
		}

		if (!pos.pseudoLegalMoveIsLegal<false, false>(move, ci.pinned))
			continue;

		ss->currentMove = move;

		pos.doMove(move, st, ci, givesCheck);
		(ss+1)->staticEvalRaw.p[0][0] = ScoreNotEvaluated;
		score = (givesCheck ? -qsearch<NT,  true>(pos, ss+1, -beta, -alpha, depth - OnePly)
				            : -qsearch<NT, false>(pos, ss+1, -beta, -alpha, depth - OnePly));
		pos.undoMove(move);

		assert(-ScoreInfinite < score && score < ScoreInfinite);

		if (score > bestScore) {
			bestScore = score;

			if (score > alpha) {

                if (PVNode) // Update pv even in fail-high case
                    updatePV(ss->pv, move, (ss+1)->pv);

				if (PVNode && score < beta) {
					alpha = score;
					bestMove = move;
				}
				else { // fail high
					tte->save(posKey, scoreToTT(score, ss->ply), BoundLower,
							  ttDepth, move, ss->staticEval, TT.generation());
					return score;
				}
			}
		}
	}

	if (INCHECK && bestScore == -ScoreInfinite)
		return matedIn(ss->ply);

	tte->save(posKey, scoreToTT(bestScore, ss->ply), 
			  ((PVNode && bestScore > oldAlpha) ? BoundExact : BoundUpper),
			  ttDepth, bestMove, ss->staticEval, TT.generation());

	assert(-ScoreInfinite < bestScore && bestScore < ScoreInfinite);

	return bestScore;
}

void checkTime() {
	const int elapsed = Time.elapsed();

    if (Limits.ponder)
		return;

    if (   (Limits.useTimeManagement() && elapsed > Time.maximum() - 10)
        || (Limits.moveTime && elapsed >= Limits.moveTime)
        || (Limits.nodes && Threads.nodes_searched() >= (uint64_t)Limits.nodes))
            Signals.stop = true;
}
} // namespace

bool RootMove::extractPonderFromTT(Position& pos)
{
    StateInfo st;
    bool ttHit;

    assert(pv.size() == 1);

    if (!pv[0].value())
        return false;

    pos.doMove(pv[0], st);
    TTEntry* tte = TT.probe(pos.getKey(), ttHit);

    if (tte != nullptr)
    {
        Move m = tte->move(); // Local copy to be SMP safe
        if (MoveList<Legal>(pos).contains(m))
            pv.push_back(m);
    }

    pos.undoMove(pv[0]);
    return pv.size() > 1;
}

#ifdef USE_extractPVFromTT
void RootMove::extractPVFromTT(Position& pos) {
	StateInfo state[MaxPly+7];
	StateInfo* st = state;
	TTEntry* tte;
	Ply ply = 0;
	Move m = pv[0];
	bool ttHit;

	assert(m && pos.moveIsPseudoLegal(m));

	pv.clear();

	do {
		pv.push_back(m);

		assert(pos.moveIsLegal(pv[ply]));
		pos.doMove(pv[ply++], *st++);
		tte = TT.probe(pos.getKey(), ttHit);
	} while (ttHit
			 // このチェックは少し無駄。駒打ちのときはmove16toMove() 呼ばなくて良い。
			 && pos.moveIsPseudoLegal(m = move16toMove(tte->move(), pos))
			 && pos.pseudoLegalMoveIsLegal<false, false>(m, pos.pinnedBB())
			 && ply < MaxPly
			 && (!pos.isDraw(20) || ply < 6));

	while (ply)
		pos.undoMove(pv[--ply]);
}
#endif