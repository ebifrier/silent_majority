#include "benchmark.hpp"
#include "common.hpp"
#include "usi.hpp"
#include "position.hpp"
#include "search.hpp"
#include "thread.hpp"

#if 1
using namespace std;

namespace {

const vector<string> Defaults = { //nanoha
  "lR1B3nl/2gp5/ngk1+BspPp/1s2p2p1/p4S3/1Pp6/P5P1P/LGG6/KN5NL b Prs5p 1",
  "5S2l/1rP2s1k1/p2+B1gnp1/5np2/3G3n1/5S2p/P1+p1PpPP1/1P1PG2KP/L2+rLPGNL b Bs3p 1",
  "lR6l/1s1g5/1k1s1+P2p/1+bpp1+Bs2/1n1n2Pp1/2P6/S2R4P/K1GG5/9 b 2NPg2l9p 1",
  "l4g1nl/4g1k2/2n1sp1p1/p5pPp/5Ps2/1P1p2s2/P1G1+p1N1P/6K2/LN5RL b RBG3Pbs3p 1",
  "1n4g1k/6r2/1+P1psg1p+L/2p1pp3/3P5/p1P1PPPP1/3SGS3/1+p1K1G2r/9 b 2BNLPs2n2l3p 1",
  "+B2+R3n1/3+L2gk1/5gss1/p1p1p1ppl/5P2p/PPPnP1PP1/3+p2N2/6K2/L4S1RL b BGS3Pgnp 1",
  "3R4l/1kg6/2ns5/spppp2+Bb/p7p/1PPPP1g2/nSNSNP2P/KG1G5/5r2L b L4Pl2p 1",
  "ln5nl/2r2gk2/1p2sgbpp/pRspppp2/L1p4PP/3PP1G2/N4PP2/3BS1SK1/5G1NL b 3P 1",
  "ln7/1r2k1+P2/p3gs3/1b1g1p+B2/1p5R1/2pPP4/PP1S1P3/2G2G3/LN1K5 b SNL3Psnl5p 1",
  "3+P3+Rl/2+P2kg2/+B2psp1p1/4p1p1p/9/2+p1P+bnKP/P6P1/4G1S2/L4G2L b G2S2NLrn5p 1",
  "ln1gb2nl/1ks4r1/1p1g4p/p1pppspB1/5p3/PPPPP1P2/1KNG1PS1P/2S4R1/L2G3NL b Pp 1",
  "lr6l/4g1k1p/1s1p1pgp1/p3P1N1P/2Pl5/PPbBSP3/6PP1/4S1SK1/1+r3G1NL b N3Pgn2p 1",
  "l1ks3+Bl/2g2+P3/p1npp4/1sNn2B2/5p2p/2PP5/PPN1P1+p1P/1KSSg2P1/L1G5+r b GL4Pr 1",
  "ln3k1nl/2P1g4/p1lpsg1pp/4p4/1p1P1p3/2SBP4/PP1G1P1PP/1K1G3+r1/LN1s2PNR b BSPp 1",
  "+N6nl/1+R2pGgk1/5Pgp1/p2p1sp2/3B1p2p/P1pP4P/6PP1/L3G1K2/7NL b RNL2Pb3s2p 1",
  "ln1g5/1r4k2/p2pppn2/2ps2p2/1p7/2P6/PPSPPPPLP/2G2K1pr/LN4G1b b BG2SLPnp 1",
  ""
};

} // namespace

void benchmark(const Position& current, istream& is) {

  string token;
  vector<string> fens;
  Search::LimitsType limits;
  std::vector<Move> moves;

  // Assign default values to missing arguments
  string ttSize    = (is >> token) ? token : "16";
  string threads   = (is >> token) ? token : "1";
  string limit     = (is >> token) ? token : "13";
  string fenFile   = (is >> token) ? token : "default";
  string limitType = (is >> token) ? token : "depth";

  Options["Hash"]    = ttSize;
  Options["Threads"] = threads;
  Search::clear();

  if (limitType == "time")
      limits.moveTime = stoi(limit); // movetime is in millisecs

  else if (limitType == "nodes")
      limits.nodes = stoi(limit);

  //else if (limitType == "mate")
  //    limits.mate = stoi(limit);

  else
      limits.depth = stoi(limit);

  if (fenFile == "default")
      fens = Defaults;

  //else if (fenFile == "current")
  //    fens.push_back(current.fen());

  else
  {
      string fen;
      ifstream file(fenFile);

      if (!file.is_open())
      {
          cerr << "Unable to open file " << fenFile << endl;
          return;
      }

      while (getline(file, fen))
          if (!fen.empty())
              fens.push_back(fen);

      file.close();
  }

  uint64_t nodes = 0;
  TimePoint elapsed = now();

  for (size_t i = 0; i < fens.size(); ++i)
  {
      Position pos(fens[i], Threads.main());

      cerr << "\nPosition: " << i + 1 << '/' << fens.size() << endl;

      /*if (limitType == "perft")
          nodes += Search::perft(pos, limits.depth * OnePly);

      else*/
      {
          Search::SearchMoves = moves;
          limits.startTime = now();
          Threads.startThinking(pos, limits, moves);
          Threads.main()->wait_for_search_finished();
          nodes += Threads.nodes_searched();
      }
  }

  elapsed = now() - elapsed + 1; // Ensure positivity to avoid a 'divide by zero'

  //dbg_print(); // Just before to exit

  cerr << "\n==========================="
       << "\nTotal time (ms) : " << elapsed
       << "\nNodes searched  : " << nodes
       << "\nNodes/second    : " << 1000 * nodes / elapsed << endl;
}
#else
// 今はベンチマークというより、PGO ビルドの自動化の為にある。
void benchmark(Position& pos) {
	std::string token;
	Search::LimitsType limits;

	std::string options[] = {"name Threads value 1",
							 "name MultiPV value 1",
							 "name OwnBook value false",
							 "name Max_Random_Score_Diff value 0"};
	for (auto& str : options) {
		std::istringstream is(str);
		setOption(is);
	}
    Search::clear();

	std::ifstream ifs("benchmark.sfen");
	std::string sfen;
	while (std::getline(ifs, sfen)) {
		std::cout << sfen << std::endl;
		std::istringstream ss_sfen(sfen);
		setPosition(pos, ss_sfen);
		std::istringstream ss_go("byoyomi 10000");
		go(pos, ss_go);
		Threads.main()->wait_for_search_finished();
	}
}
#endif