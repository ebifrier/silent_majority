#ifndef APERY_THREAD_HPP
#define APERY_THREAD_HPP

#include "common.hpp"
#include "evaluate.hpp"
#include "usi.hpp"
#include "tt.hpp"
#include "search.hpp"
#include "movePicker.hpp"

class Thread {

  std::thread nativeThread;
  Mutex mutex;
  ConditionVariable sleepCondition;
  bool exit, searching;

public:
  Thread();
  virtual ~Thread();
  virtual void search();
  void idle_loop();
  void start_searching(bool resume = false);
  void wait_for_search_finished();
  void wait(std::atomic_bool& b);

    size_t pvIdx;
	size_t idx;
    int maxPly, callsCnt;

    Position rootPos;
    Search::RootMoves rootMoves;
    Depth rootDepth;
    HistoryStats history;
    MoveStats counterMoves;
    Depth completedDepth;
    std::atomic_bool resetCalls;
};

struct MainThread : public Thread {
  virtual void search();

  bool easyMovePlayed, failedLow;
  double bestMoveChanges;
  Score previousScore;
};

struct ThreadPool : public std::vector<Thread*> {

	void init();
	void exit();

	MainThread* main() { return static_cast<MainThread*>(at(0)); }
	void startThinking(const Position& pos, const Search::LimitsType& limits, const std::vector<Move>& searchMoves);
    void readUSIOptions();
    int64_t nodes_searched();
};

extern ThreadPool Threads;

#endif // #ifndef APERY_THREAD_HPP
