#include "common.hpp"
#include "search.hpp"
#include "usi.hpp"
#include "timeManager.hpp"
#include <cfloat>

TimeManagement Time; // Our global time management object

namespace {

  enum TimeType { OptimumTime, MaxTime };

  const int MoveHorizon   = 50;   // Plan time management at most this many moves ahead
  const double MaxRatio   = 7.09;  // When in trouble, we can step over reserved time with this ratio
  const double StealRatio = 0.35; // However we must not steal time from remaining moves over this ratio


  // move_importance() is a skew-logistic function based on naive statistical
  // analysis of "how many games are still undecided after n half-moves". Game
  // is considered "undecided" as long as neither side has >275cp advantage.
  // Data was extracted from CCRL game database with some simple filtering criteria.

  double move_importance(int ply) {

    const double XScale = 7.64;
    const double XShift = 58.4;
    const double Skew   = 0.183;

    return pow((1 + exp((ply - XShift) / XScale)), -Skew) + DBL_MIN; // Ensure non-zero
  }

	template <TimeType T> int remaining(const int myTime, const int movesToGo, const Ply ply, const int slowMover) {
		const double TMaxRatio   = (T == OptimumTime ? 1 : MaxRatio);
		const double TStealRatio = (T == OptimumTime ? 0 : StealRatio);

		const double thisMoveImportance = double(move_importance(ply) * slowMover) / 100;
		double otherMoveImportance = 0;

		for (int i = 1; i < movesToGo; ++i)
			otherMoveImportance += move_importance(ply + 2 * i);

		const double ratio1 =
			(TMaxRatio * thisMoveImportance) / (TMaxRatio * thisMoveImportance + otherMoveImportance);
		const double ratio2 =
			(thisMoveImportance + TStealRatio * otherMoveImportance) / (thisMoveImportance + otherMoveImportance);

		return static_cast<int>(myTime * std::min(ratio1, ratio2));
	}
} // namespace

/// init() is called at the beginning of the search and calculates the allowed
/// thinking time out of the time control and current game ply. We support four
/// different kinds of time controls, passed in 'limits':
///
///  inc == 0 && movestogo == 0 means: x basetime  [sudden death!]
///  inc == 0 && movestogo != 0 means: x moves in y minutes
///  inc >  0 && movestogo == 0 means: x basetime + z increment
///  inc >  0 && movestogo != 0 means: x moves in y minutes + z increment

void TimeManagement::init(Search::LimitsType& limits, Color us, int ply) {
	const int minThinkingTime = Options["Minimum_Thinking_Time"];
    const int slowMover       = Options["Slow_Mover"];
    const int moveOverhead    = Options["Move_Overhead"];
    const int npmsec          = Options["nodestime"];

    if (npmsec)
    {
      if (!availableNodes) // Only once at game start
        availableNodes = npmsec * limits.time[us]; // Time is in msec

      // Convert from millisecs to nodes
      limits.time[us] = (int)availableNodes;
      limits.inc[us] *= npmsec;
      limits.npmsec = npmsec;
    }

    startTime = limits.startTime;
	optimumTime = maximumTime = std::max(limits.time[us], minThinkingTime);

    const int MaxMTG = limits.movesToGo ? std::min(limits.movesToGo, MoveHorizon) : MoveHorizon;

    for (int hypMTG = 1; hypMTG <= MaxMTG; ++hypMTG) {
		int hypMyTime =
			limits.time[us]
			+ limits.inc[us] * (hypMTG - 1)
            - moveOverhead * (2 + std::min(hypMTG, 40));

		hypMyTime = std::max(hypMyTime, 0);

		const int t1 = minThinkingTime + remaining<OptimumTime>(hypMyTime, hypMTG, ply, slowMover);
		const int t2 = minThinkingTime + remaining<MaxTime>(hypMyTime, hypMTG, ply, slowMover);

		optimumTime = std::min(optimumTime, t1);
		maximumTime = std::min(maximumTime, t2);
	}

	if (Options["USI_Ponder"])
		optimumTime += optimumTime / 4;

#if 1 ///Apery
	// こちらも minThinkingTime 以上にする。
	optimumTime = std::max(optimumTime, minThinkingTime);
	optimumTime = std::min(optimumTime, maximumTime);

	if (limits.moveTime != 0) {
		if (optimumTime < limits.moveTime)
			optimumTime = std::min(limits.time[us], limits.moveTime);
		if (maximumTime < limits.moveTime)
			maximumTime = std::min(limits.time[us], limits.moveTime);
		optimumTime += limits.moveTime;
		maximumTime += limits.moveTime;
		if (limits.time[us] != 0)
			limits.moveTime = 0;
	}
#endif
	SYNCCOUT << "info string optimum_search_time = " << optimumTime << SYNCENDL;
	SYNCCOUT << "info string maximum_search_time = " << maximumTime << SYNCENDL;
}
