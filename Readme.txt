概要

SILENT_MAJORITYはAperyを改造した USI プロトコルの将棋エンジンです。
USI エンジンに対応した GUI ソフトを使って下さい。
将棋所 以外で動作検証しておりませんので、将棋所 の使用を推奨します。
SILENT_MAJORITY は GNU General Public License version 3 またはそれ以降のバージョンのもとで配布されます。
主にチェスエンジンの Stockfish の実装を参考にしています。
将棋固有のデータ構造、評価関数等、Bonanza の実装を非常によく参考にしています。

ファイルの説明

・Readme.txt, このファイルです。
・Copying.txt, GNU General Public License version 3 条文です。
・bin/, 評価関数や定跡のバイナリのフォルダです。


利用環境

メモリに最低でも 1GB 程度空きがあること。
64bit OS であること。


使い方

将棋所での使い方のみを説明します。
将棋所を立ち上げます。

Windows の場合
Shogidokoro.exe をダブルクリックして下さい。
立ち上がらない場合は、.NET Framework が古い可能性が高いです。新しいものにして下さい。

将棋所のエンジン登録で Windows の場合は SILENT_MAJORITY*_x64.exe を登録して下さい。
(いくつかの実行ファイルがありますが、ご使用のCPUで使えるものを選んでください)
一度、「これは USI エンジンではありません。」といったポップアップが表示されるかも知れません。
タイムアウトして登録に失敗している可能性があるので、もう一度エンジン登録してみて下さい。
それでも登録に失敗するなら、SILENT_MAJORITY が正しく動作していない可能性があります。
SILENT_MAJORITY_x64.exe をダブルクリックして、usi とコマンドを打ってみて下さい。
usiok が表示されない場合は、ご利用の PC では SILENT_MAJORITY が動作しないようです。

将棋所に登録出来ましたら、後は将棋所の使い方を参照して下さい。
