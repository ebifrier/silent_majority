#include "usi.hpp"
#include "position.hpp"
#include "move.hpp"
#include "movePicker.hpp"
#include "generateMoves.hpp"
#include "search.hpp"
#include "tt.hpp"
#include "book.hpp"
#include "thread.hpp"
#include "benchmark.hpp"
#include "learner.hpp"

#include "timeManager.hpp"

using namespace std;

USI::OptionsMap Options; // Global object

namespace {
	// 論理的なコア数の取得
	inline int cpuCoreCount() {
        // todo: boost::thread::physical_concurrency() を使うこと。
		// std::thread::hardware_concurrency() は 0 を返す可能性がある。
		return std::max(static_cast<int>(std::thread::hardware_concurrency()/2), 1);
	}

	class StringToPieceTypeCSA : public std::map<std::string, PieceType> {
	public:
		StringToPieceTypeCSA() {
			(*this)["FU"] = Pawn;
			(*this)["KY"] = Lance;
			(*this)["KE"] = Knight;
			(*this)["GI"] = Silver;
			(*this)["KA"] = Bishop;
			(*this)["HI"] = Rook;
			(*this)["KI"] = Gold;
			(*this)["OU"] = King;
			(*this)["TO"] = ProPawn;
			(*this)["NY"] = ProLance;
			(*this)["NK"] = ProKnight;
			(*this)["NG"] = ProSilver;
			(*this)["UM"] = Horse;
			(*this)["RY"] = Dragon;
		}
		PieceType value(const std::string& str) const {
			return this->find(str)->second;
		}
		bool isLegalString(const std::string& str) const {
			return (this->find(str) != this->end());
		}
	};
	const StringToPieceTypeCSA g_stringToPieceTypeCSA;
}

namespace USI {
	void onThreads(const Option&)      { Threads.readUSIOptions(); }
	void onHashSize(const Option& opt) { TT.resize(opt); }
    void onClearHash(const Option&)    { Search::clear();}
	void onEvalDir(const Option& opt)    {
		std::unique_ptr<Evaluater>(new Evaluater)->init(opt, true);
	}

bool CaseInsensitiveLess::operator () (const std::string& s1, const std::string& s2) const {
	for (size_t i = 0; i < s1.size() && i < s2.size(); ++i) {
		const int c1 = tolower(s1[i]);
		const int c2 = tolower(s2[i]);
		if (c1 != c2)
			return c1 < c2;
	}
	return s1.size() < s2.size();
}

void init(OptionsMap& o) {
	o["USI_Hash"]                    = Option(256, 1, 65536, onHashSize);
	o["Clear_Hash"]                  = Option(onClearHash);
	o["Book_File"]                   = Option("book/20150503/book.bin");
	o["Best_Book_Move"]              = Option(false);
	o["OwnBook"]                     = Option(true);
	o["Min_Book_Ply"]                = Option(SHRT_MAX, 0, SHRT_MAX);
	o["Max_Book_Ply"]                = Option(SHRT_MAX, 0, SHRT_MAX);
	o["Min_Book_Score"]              = Option(-180, -ScoreInfinite, ScoreInfinite);
	o["Eval_Dir"]                    = Option("20160307", onEvalDir);
	o["Write_Synthesized_Eval"]      = Option(false);
	o["USI_Ponder"]                  = Option(true);
	o["Byoyomi_Margin"]              = Option(500, 0, INT_MAX);
    o["Inc_Margin"]                  = Option(4500, 0, INT_MAX);
	o["MultiPV"]                     = Option(1, 1, MaxLegalMoves);
	o["Skill_Level"]                 = Option(20, 0, 20);
	o["Max_Random_Score_Diff"]       = Option(0, 0, ScoreMate0Ply);
	o["Max_Random_Score_Diff_Ply"]   = Option(40, 0, SHRT_MAX);
	o["Slow_Mover"]                  = Option(100, 10, 1000);
	o["Minimum_Thinking_Time"]       = Option(1500, 0, INT_MAX);
	o["Threads"]                     = Option(cpuCoreCount(), 1, 128, onThreads);
    o["Move_Overhead"] = Option(30, 0, 5000);
    o["nodestime"] = Option(0, 0, 10000);
#ifdef RESIGN
    o["Resign"] = Option(2000, 0, 10000);
#endif
}

Option::Option(const char* v, Fn* f) :
	type_("string"), min_(0), max_(0), onChange_(f)
{
	defaultValue_ = currentValue_ = v;
}

Option::Option(const bool v, Fn* f) :
	type_("check"), min_(0), max_(0), onChange_(f)
{
	defaultValue_ = currentValue_ = (v ? "true" : "false");
}

Option::Option(Fn* f) :
	type_("button"), min_(0), max_(0), onChange_(f)
{}

Option::Option(const int v, const int min, const int max, Fn* f)
	: type_("spin"), min_(min), max_(max), onChange_(f)
{
	std::ostringstream ss;
	ss << v;
	defaultValue_ = currentValue_ = ss.str();
}

Option& Option::operator = (const std::string& v) {
	assert(!type_.empty());

	if ((type_ != "button" && v.empty())
		|| (type_ == "check" && v != "true" && v != "false")
		|| (type_ == "spin" && (atoi(v.c_str()) < min_ || max_ < atoi(v.c_str()))))
	{
		return *this;
	}

	if (type_ != "button")
		currentValue_ = v;

	if (onChange_ != nullptr)
		(*onChange_)(*this);

	return *this;
}

std::ostream& operator << (std::ostream& os, const OptionsMap& om) {
	for (auto& elem : om) {
		const Option& o = elem.second;
		os << "\noption name " << elem.first << " type " << o.type_;
		if (o.type_ != "button")
			os << " default " << o.defaultValue_;

		if (o.type_ == "spin")
			os << " min " << o.min_ << " max " << o.max_;
	}
	return os;
}

} // namespace USI

void go(const Position& pos, std::istringstream& ssCmd) {
	Search::LimitsType limits;
	std::vector<Move> moves;
	std::string token;

    limits.startTime = now(); // As early as possible!

	while (ssCmd >> token) {
		if      (token == "ponder"     ) limits.ponder = true;
		else if (token == "btime"      ) ssCmd >> limits.time[Black];
		else if (token == "wtime"      ) ssCmd >> limits.time[White];
        else if (token == "binc"       ) ssCmd >> limits.inc[Black];
        else if (token == "winc"       ) ssCmd >> limits.inc[White];
		else if (token == "infinite"   ) limits.infinite = true;
		else if (token == "byoyomi" || token == "movetime") { ssCmd >> limits.moveTime; }
		else if (token == "depth"      ) { ssCmd >> limits.depth; }
		else if (token == "nodes"      ) { ssCmd >> limits.nodes; }
		else if (token == "searchmoves") {
			while (ssCmd >> token)
				moves.push_back(usiToMove(pos, token));
		}
	}
    if (limits.moveTime != 0)
        limits.moveTime -= Options["Byoyomi_Margin"];
    else if (limits.inc[pos.turn()] != 0)
        limits.time[pos.turn()] -= Options["Inc_Margin"];

	Search::SearchMoves = moves;
	Threads.startThinking(pos, limits, moves);
}

#if defined LEARN
// 学習用。通常の go 呼び出しは文字列を扱って高コストなので、大量に探索の開始、終了を行う学習では別の呼び出し方にする。
void go(const Position& pos, const Ply depth, const Move move) {
	LimitsType limits;
	std::vector<Move> moves;
	limits.depth = depth;
	moves.push_back(move);
	pos.searcher()->threads.startThinking(pos, limits, moves);
}
#endif

Move usiToMoveBody(const Position& pos, const std::string& moveStr) {
	Move move;
	if (g_charToPieceUSI.isLegalChar(moveStr[0])) {
		// drop
		const PieceType ptTo = pieceToPieceType(g_charToPieceUSI.value(moveStr[0]));
		if (moveStr[1] != '*')
			return Move::moveNone();
		const File toFile = charUSIToFile(moveStr[2]);
		const Rank toRank = charUSIToRank(moveStr[3]);
		if (!isInSquare(toFile, toRank))
			return Move::moveNone();
		const Square to = makeSquare(toFile, toRank);
		move = makeDropMove(ptTo, to);
	}
	else {
		const File fromFile = charUSIToFile(moveStr[0]);
		const Rank fromRank = charUSIToRank(moveStr[1]);
		if (!isInSquare(fromFile, fromRank))
			return Move::moveNone();
		const Square from = makeSquare(fromFile, fromRank);
		const File toFile = charUSIToFile(moveStr[2]);
		const Rank toRank = charUSIToRank(moveStr[3]);
		if (!isInSquare(toFile, toRank))
			return Move::moveNone();
		const Square to = makeSquare(toFile, toRank);
		if (moveStr[4] == '\0')
			move = makeNonPromoteMove<Capture>(pieceToPieceType(pos.piece(from)), from, to, pos);
		else if (moveStr[4] == '+') {
			if (moveStr[5] != '\0')
				return Move::moveNone();
			move = makePromoteMove<Capture>(pieceToPieceType(pos.piece(from)), from, to, pos);
		}
		else
			return Move::moveNone();
	}

	if (pos.moveIsPseudoLegal(move, true)
		&& pos.pseudoLegalMoveIsLegal<false, false>(move, pos.pinnedBB()))
	{
		return move;
	}
	return Move::moveNone();
}
#if !defined NDEBUG
// for debug
Move usiToMoveDebug(const Position& pos, const std::string& moveStr) {
	for (MoveList<LegalAll> ml(pos); !ml.end(); ++ml) {
		if (moveStr == ml.move().toUSI())
			return ml.move();
	}
	return Move::moveNone();
}
Move csaToMoveDebug(const Position& pos, const std::string& moveStr) {
	for (MoveList<LegalAll> ml(pos); !ml.end(); ++ml) {
		if (moveStr == ml.move().toCSA())
			return ml.move();
	}
	return Move::moveNone();
}
#endif
Move usiToMove(const Position& pos, const std::string& moveStr) {
	const Move move = usiToMoveBody(pos, moveStr);
	assert(move == usiToMoveDebug(pos, moveStr));
	return move;
}

Move csaToMoveBody(const Position& pos, const std::string& moveStr) {
	if (moveStr.size() != 6)
		return Move::moveNone();
	const File toFile = charCSAToFile(moveStr[2]);
	const Rank toRank = charCSAToRank(moveStr[3]);
	if (!isInSquare(toFile, toRank))
		return Move::moveNone();
	const Square to = makeSquare(toFile, toRank);
	const std::string ptToString(moveStr.begin() + 4, moveStr.end());
	if (!g_stringToPieceTypeCSA.isLegalString(ptToString))
		return Move::moveNone();
	const PieceType ptTo = g_stringToPieceTypeCSA.value(ptToString);
	Move move;
	if (moveStr[0] == '0' && moveStr[1] == '0')
		// drop
		move = makeDropMove(ptTo, to);
	else {
		const File fromFile = charCSAToFile(moveStr[0]);
		const Rank fromRank = charCSAToRank(moveStr[1]);
		if (!isInSquare(fromFile, fromRank))
			return Move::moveNone();
		const Square from = makeSquare(fromFile, fromRank);
		PieceType ptFrom = pieceToPieceType(pos.piece(from));
		if (ptFrom == ptTo)
			// non promote
			move = makeNonPromoteMove<Capture>(ptFrom, from, to, pos);
		else if (ptFrom + PTPromote == ptTo)
			// promote
			move = makePromoteMove<Capture>(ptFrom, from, to, pos);
		else
			return Move::moveNone();
	}

	if (pos.moveIsPseudoLegal(move, true)
		&& pos.pseudoLegalMoveIsLegal<false, false>(move, pos.pinnedBB()))
	{
		return move;
	}
	return Move::moveNone();
}
Move csaToMove(const Position& pos, const std::string& moveStr) {
	const Move move = csaToMoveBody(pos, moveStr);
	assert(move == csaToMoveDebug(pos, moveStr));
	return move;
}

void setPosition(Position& pos, std::istringstream& ssCmd) {
	std::string token;
	std::string sfen;

	ssCmd >> token;

	if (token == "startpos") {
		sfen = DefaultStartPositionSFEN;
		ssCmd >> token; // "moves" が入力されるはず。
	}
	else if (token == "sfen") {
		while (ssCmd >> token && token != "moves")
			sfen += token + " ";
	}
	else
		return;

	pos.set(sfen, Threads.main());
	Search::SetUpStates = StateStackPtr(new std::stack<StateInfo>());

	Ply currentPly = pos.gamePly();
	while (ssCmd >> token) {
		const Move move = usiToMove(pos, token);
		if (move.isNone()) break;
		Search::SetUpStates->push(StateInfo());
		pos.doMove(move, Search::SetUpStates->top());
		++currentPly;
	}
	pos.setStartPosPly(currentPly);
}

void setOption(std::istringstream& ssCmd) {
	std::string token;
	std::string name;
	std::string value;

	ssCmd >> token; // "name" が入力されるはず。

	ssCmd >> name;
	// " " が含まれた名前も扱う。
	while (ssCmd >> token && token != "value")
		name += " " + token;

	ssCmd >> value;
	// " " が含まれた値も扱う。
	while (ssCmd >> token)
		value += " " + token;

    if (!Options.count(name))
      cout << "No such option: " << name << endl;

    else if (value.empty()) // UCI buttons don't have a value
      Options[name] = true;

    else
      Options[name] = value;
}

#if !defined MINIMUL
// for debug
// 指し手生成の速度を計測
void measureGenerateMoves(const Position& pos) {
	pos.print();

	MoveStack legalMoves[MaxLegalMoves];
	for (int i = 0; i < MaxLegalMoves; ++i) legalMoves[i].move = moveNone();
	MoveStack* pms = &legalMoves[0];
	const u64 num = 5000000;
    Time_ t = Time_::currentTime();
	if (pos.inCheck()) {
		for (u64 i = 0; i < num; ++i) {
			pms = &legalMoves[0];
			pms = generateMoves<Evasion>(pms, pos);
		}
	}
	else {
		for (u64 i = 0; i < num; ++i) {
			pms = &legalMoves[0];
			pms = generateMoves<CapturePlusPro>(pms, pos);
			pms = generateMoves<NonCaptureMinusPro>(pms, pos);
			pms = generateMoves<Drop>(pms, pos);
//			pms = generateMoves<PseudoLegal>(pms, pos);
//			pms = generateMoves<Legal>(pms, pos);
		}
	}
	const int elapsed = t.elapsed();
	std::cout << "elapsed = " << elapsed << " [msec]" << std::endl;
	if (elapsed != 0)
		std::cout << "times/s = " << num * 1000 / elapsed << " [times/sec]" << std::endl;
	const ptrdiff_t count = pms - &legalMoves[0];
	std::cout << "num of moves = " << count << std::endl;
	for (int i = 0; i < count; ++i)
		std::cout << legalMoves[i].move.toCSA() << ", ";
	std::cout << std::endl;
}
#endif

#ifdef NDEBUG
const std::string MyName = "SILENT_MAJORITY 1.2";
#else
const std::string MyName = "Apery Debug Build";
#endif

void USI::loop(int argc, char* argv[]) {
	Position pos(DefaultStartPositionSFEN, Threads.main());

	std::string cmd;
	std::string token;

#if defined MPI_LEARN
	boost::mpi::environment  env(argc, argv);
	boost::mpi::communicator world;
	if (world.rank() != 0) {
		learn(pos, env, world);
		return;
	}
#endif

	for (int i = 1; i < argc; ++i)
		cmd += std::string(argv[i]) + " ";

	do {
		if (argc == 1 && !std::getline(std::cin, cmd))
			cmd = "quit";

		std::istringstream ssCmd(cmd);

		ssCmd >> std::skipws >> token;

		if (token == "quit" || token == "stop" || token == "ponderhit" || token == "gameover") {
			if (token != "ponderhit" || Search::Signals.stopOnPonderhit) {
              Search::Signals.stop = true;
				Threads.main()->start_searching(true);
			}
			else
				Search::Limits.ponder = false;
			if (token == "ponderhit" && Search::Limits.moveTime != 0)
              Search::Limits.moveTime += Time.elapsed();
		}
		else if (token == "usinewgame") {
            Search::clear();
            Time.availableNodes = 0;
#if defined INANIWA_SHIFT
			inaniwaFlag = NotInaniwa;
#endif
			for (int i = 0; i < 100; ++i) g_randomTimeSeed(); // 最初は乱数に偏りがあるかも。少し回しておく。
		}
		else if (token == "usi"      ) SYNCCOUT << "id name " << MyName
												<< "\nid author Hiraoka Takuya , T. Romstad, M. Costalba, J. Kiiski, G. Linscott and more"
												<< "\n" << Options
												<< "\nusiok" << SYNCENDL;
		else if (token == "go"       ) go(pos, ssCmd);
		else if (token == "isready"  ) SYNCCOUT << "readyok" << SYNCENDL;
		else if (token == "position" ) setPosition(pos, ssCmd);
		else if (token == "setoption") setOption(ssCmd);
#if defined LEARN
		else if (token == "l"        ) {
			auto learner = std::unique_ptr<Learner>(new Learner);
#if defined MPI_LEARN
			learner->learn(pos, env, world);
#else
			learner->learn(pos, ssCmd);
#endif
		}
#endif
#if !defined MINIMUL
		// 以下、デバッグ用
		else if (token == "bench"    ) benchmark(pos, ssCmd);
		else if (token == "key"      ) SYNCCOUT << pos.getKey() << SYNCENDL;
		else if (token == "d"        ) pos.print();
		else if (token == "s"        ) measureGenerateMoves(pos);
		else if (token == "t"        ) std::cout << pos.mateMoveIn1Ply().toCSA() << std::endl;
		else if (token == "b"        ) makeBook(pos, ssCmd);
#endif
		else                           SYNCCOUT << "unknown command: " << cmd << SYNCENDL;
	} while (token != "quit" && argc == 1);

	if (Options["Write_Synthesized_Eval"])
		Evaluater::writeSynthesized(Options["Eval_Dir"]);

	Threads.main()->wait_for_search_finished();
}
