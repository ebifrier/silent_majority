#include "generateMoves.hpp"
#include "search.hpp"
#include "thread.hpp"
#include "usi.hpp"

using namespace Search;

ThreadPool Threads; // Global object

Thread::Thread() {

  resetCalls = exit = false;
  maxPly = callsCnt = 0;
  history.clear();
  counterMoves.clear();
  idx = Threads.size(); // Start from 0

  std::unique_lock<Mutex> lk(mutex);
  searching = true;
  nativeThread = std::thread(&Thread::idle_loop, this);
  sleepCondition.wait(lk, [&] { return !searching; });
}


/// Thread destructor waits for thread termination before returning

Thread::~Thread() {

  mutex.lock();
  exit = true;
  sleepCondition.notify_one();
  mutex.unlock();
  nativeThread.join();
}


/// Thread::wait_for_search_finished() waits on sleep condition
/// until not searching

void Thread::wait_for_search_finished() {

  std::unique_lock<Mutex> lk(mutex);
  sleepCondition.wait(lk, [&] { return !searching; });
}


/// Thread::wait() waits on sleep condition until condition is true

void Thread::wait(std::atomic_bool& condition) {

  std::unique_lock<Mutex> lk(mutex);
  sleepCondition.wait(lk, [&] { return bool(condition); });
}


/// Thread::start_searching() wakes up the thread that will start the search

void Thread::start_searching(bool resume) {

  std::unique_lock<Mutex> lk(mutex);

  if (!resume)
    searching = true;

  sleepCondition.notify_one();
}

void Thread::idle_loop() {
#ifdef Handle_Windows_Processors_Groups
	WinProcGroup::bindThisThread(idx);
#endif
  while (!exit)
  {
    std::unique_lock<Mutex> lk(mutex);

    searching = false;

    while (!searching && !exit)
    {
      sleepCondition.notify_one(); // Wake up any waiting thread
      sleepCondition.wait(lk);
    }

    lk.unlock();

    if (!exit)
      search();
  }
}

void ThreadPool::init() {
  push_back(new MainThread);
	readUSIOptions();
}

void ThreadPool::exit() {
  while (size())
    delete back(), pop_back();
}

void ThreadPool::readUSIOptions() {
	const size_t requested   = Options["Threads"];

	assert(0 < requested);

	while (size() < requested)
      push_back(new Thread);

	while (requested < size()) {
      delete back(), pop_back();
	}
}

uint64_t ThreadPool::nodes_searched() {

  uint64_t nodes = 0;
  for (Thread* th : *this)
      nodes += th->rootPos.nodesSearched();
  return nodes;
}

void ThreadPool::startThinking(const Position& pos, const Search::LimitsType& limits,
							   const std::vector<Move>& searchMoves)
{
#if defined LEARN
#else
  main()->wait_for_search_finished();
#endif

  Search::Signals.stopOnPonderhit = Search::Signals.stop = false;
  Search::Limits = limits;

    main()->rootMoves.clear();
    main()->rootPos = pos;

#if defined LEARN
	// searchMoves を直接使う。
	RootMoves.push_back(RootMove(searchMoves[0]));
#else
	const MoveType MT = Legal;
	for (MoveList<MT> ml(pos); !ml.end(); ++ml) {
		if (searchMoves.empty()
			|| std::find(searchMoves.begin(), searchMoves.end(), ml.move()) != searchMoves.end())
		{
          main()->rootMoves.push_back(RootMove(ml.move()));
		}
	}
#endif

#if defined LEARN
	// 浅い探索なので、thread 生成、破棄のコストが高い。余分な thread を生成せずに直接探索を呼び出す。
	Search::think();
#else

	main()->start_searching();
#endif
}

