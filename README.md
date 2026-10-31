# SPRESENSE + MCP25625 最小動作確認

SPRESENSE 拡張ボードと [MCP25625-breakout](https://www.switch-science.com/products/7228) 用の PlatformIO / Arduino プロジェクトです。MCP25625 は MCP2515 互換として `autowp-mcp2515 1.3.1` で制御します。CAN は **500 kbps**、ボードの発振子は **16 MHz**、SPI は 1 MHz / Mode 0 です。

## 配線

信号配線は以下のとおりです。当初の添付図から CS と RESET を入れ替えています。電源を切って接続してください。

| SPRESENSE 拡張ボード | MCP25625-breakout |
| --- | --- |
| D13 | SCK |
| D12 | MISO |
| D11 | MOSI |
| D10 | 未接続 |
| D9 | CS |
| D8 | INT（Low 有効） |
| D7 | RST / RESET（Low でリセット） |
| 5 V | VIO |
| 5 V | VDDA |
| GND | GND |

**拡張ボードの JP1 を 5 V に設定し、VIO・VDDA ともに 5 V を供給してください。** VIO は SPI / RESET / INT の信号電圧、VDDA はトランシーバー電源です。本プロジェクトでは信号電圧も 5 V に統一します。電圧の設定は配線と JP1 で行うため、ソースコードの変更は不要です。

D10 には接続しません。SPRESENSE の SPI はハードウェア CS を操作しますが、本コードは別途 D9 をソフトウェアで操作します。INT は割り込みハンドラーを使わず `loop()` でポーリングします。

## 1. 1台で内部ループバック確認（標準設定）

プロジェクトのディレクトリーで以下を実行します。`pio` が PATH にない場合は `~/.platformio/penv/bin/pio` を使用してください。

```sh
pio run -e spresense
pio run -e spresense -t upload
pio device monitor -b 115200
```

書き込みには SPRESENSE メインボード側の USB を使用します。ポートが複数ある場合は `pio device list` で確認し、書き込みに `--upload-port <ポート>`、モニターに `--port <ポート>` を指定してください。起動ログを見逃した場合はモニターを開いてからリセットしてください。

CANH / CANL に接続する相手機器は不要です。1秒ごとに標準 ID `0x123`、8バイトを内部で送受信し、ID・長さ・内容を比較します。先頭バイトは毎回増加し、255 の次は 0 になります。

```text
SPRESENSE + MCP25625 / 16 MHz / CAN 500 kbps
LOOPBACK ready: expect TX, RX and LOOPBACK PASS every second.
TX queued ID=0x123
RX ID=0x123 STD DLC=8 DATA=00 11 22 33 44 55 66 77
LOOPBACK PASS
```

`LOOPBACK PASS` が繰り返されれば SPI と MCP25625 内部の送受信が確認できます。外部の CAN 配線・トランシーバーの動作は次の手順で確認します。

## 2. 外部 CAN バスで確認

500 kbps の通常モードで動く別の CAN ノード（別ボードや USB-CAN など）を接続します。CANH 同士、CANL 同士、GND 同士を接続し、バスの両端だけに 120 Ω の終端抵抗を入れます。この基板が端の場合は終端スイッチを USE、途中の場合は NC にします。相手側の内蔵終端も確認してください。

```sh
pio run -e spresense_can
pio run -e spresense_can -t upload
pio device monitor -b 115200
```

1秒ごとに ID `0x123` を送信します。相手側で受信できることを確認し、相手から任意の CAN ID（例: `0x321`）を送信すると SPRESENSE 側に `RX ...` が表示されます。両方に同じコードを書き込んでも確認できます。通常モードでは自分の送信は内部受信されません。

`TX queued` は送信バッファーへの登録を表し、バス上での送信完了や相手の受信を保証しません。送信成功には ACK を返す相手が必要です。相手が未接続、listen-only、通信速度不一致の場合は再送が続き、送信バッファーが埋まって `TX FAIL` になることがあります。配線・相手を修正後にリセットしてください。`EFLG=0x0` はエラーフラグなしですが、これだけでも通信成功の証明にはなりません。

## 3. B-G431B-ESC1（SimpleFOC_B-G431）を CAN で操作する

モータボード B-G431B-ESC1 の firmware（[SimpleFOC_B-G431](https://github.com/sasa3-dev/SimpleFOC_B-G431) の `can-75pct` ブランチ以降）に CAN でコマンドを送り、その `how_to_jump.md` の手順（原点設定 → 最高速度の設定 → 出力 1 回転 → 正規化 の繰り返し）を実行します。各回の到達時刻などは、ボードが流す 1 kHz のテレメトリから測ります。

```sh
pio run -e spresense_bg431
pio run -e spresense_bg431 -t upload
pio device monitor -b 115200
```

### バスの設定

| 項目 | 値 |
| --- | --- |
| 通信速度 | 1 Mbps（クラシック CAN 2.0、標準 ID） |
| サンプル点 | 75%。MCP25625 は CNF1..3 = `40 91 01`（1 ビット 8 tq：Sync 1 + Prop 2 + PS1 3 + PS2 2、SJW 2）。ライブラリの `CAN_1000KBPS` は 62.5% なので使わず、レジスタを直接書いて読み戻す |
| SPI | 8 MHz（`BG431_SPI_HZ`）。起動時に MCP25625 の送信バッファへの書き込みと読み戻しを 64 回試し、合わなければ半分の速度に落とす |
| 受信フィルタ | RXB0：テレメトリ `0x101`/`0x111`/`0x121`/`0x131`（溢れたら RXB1 へ）、RXB1：イベント `0x201`・結果 `0x281`・状態 `0x701`。コンソール文字（`0x581`）などはハードウェアで捨てる |

ボードは 1 フレームを約 0.2 ms おきに送り、MCP25625 は受信フレームを 2 つしか溜められません。SPI を 1 MHz のままにすると、1 フレームの読み出し（ライブラリの `readMessage()` で SPI 5 回・約 26 バイト）が間に合わず、テレメトリを取りこぼします。`loop()` では `delay()` を使わず、受信を常に回します。

CANH・CANL・GND をボードの CAN 端子につなぎます。ボードは firmware で 120 Ω の終端を入れているので、この基板がバスの反対の端なら終端スイッチを USE にします。USB2CAN なども同じバスにつなぐ場合、終端はバスの両端の 2 か所だけにします。

### 操作（シリアルモニター、1 行ずつ）

| 入力 | 動作 |
| --- | --- |
| `jump [n]` | 原点を現在位置に設定し、最高速度の設定（`A10000` `V460` `MVR1000` `MVP0.05`）をして、「`H-86.5823,2500`（負方向に出力 1 回転）→ `GN`」を n 回（既定 3）。最後に既定値（`A1000` `V145` `MVR150` `MVP0.02`）に戻す |
| `jumpk [n]` | 原点設定を飛ばして同じことをする |
| `c <コマンド>` | ボードのコマンドを 1 つ送って結果を表示（例 `c Q`、`c MVP`、`c ME0`） |
| `s` | 非常停止フレームを送る |
| `stat` | SPI 速度、受信したテレメトリの数、欠落、受信バッファのオーバーフロー回数 |

`jump` の実行中に何かキーを送ると非常停止します。次のときは駆動を止めて（`ME0`）残りを中止します：`H` の結果が OK / TIMEOUT 以外、最終誤差 0.2 rad 超、電流ピーク 11 A 超、ボードの再起動。キープアライブ（`0x711`）は 100 ms ごとに自動で送ります。途切れて 0.5 秒たつとボードがモータを止めます。

重力がかかる構成では、原点設定の間（velocity モードの速度 0 保持）に軸が回り、原点が少しずれます。正確に取るなら出力軸を押さえてから `jump` を送ってください。

**シリアルモニターを開くと SPRESENSE がリセットされます**（USB シリアルの DTR/RTS）。その間キープアライブが途切れるので、ボードの見張り（0.5 秒）がモータを無効にします。`jump` は手順 1 で `ME1` を送るのでそのまま動きますが、`c` で単独のコマンドを送る前には `c G` → `c ME1` で有効に戻してください。

### 実績（2026-11-01、負荷あり・負方向）

`jump 3`：3 回とも OK（約 0.7 秒で静定）、到達 241.1〜245.0 ms（4.08〜4.15 rps）、行き過ぎ 0.7〜1.6°（出力軸）、電流ピーク 8.2〜8.8 A。1 回転の最中の 1 kHz テレメトリは欠けなし（静定までの時間 + 1 サンプル）。SPI 8 MHz の自己診断は合格。

受信はライブラリの `readMessage()`（SPI 5 回）ではなく、READ STATUS と READ RX BUFFER の 2 回で 1 フレームを読みます。コマンドの合間に結果を USB シリアルへ表示している間は `poll()` が最大約 2 ms 空き、その間に受信バッファが溢れることがあります（3 回の実行で、50 Hz のテレメトリ 1 サンプル）。`stat` の `RX overflows` と `max poll gap` で確認できます。

### ファイル

- `include/bg431_protocol.h`：ボードの CAN の取り決め（ID、イベント、結果の状態コード、テレメトリの形式）。SimpleFOC_B-G431 の `include/can_link.h` と `include/cmd_results.h` を写したもの。変えたら両方そろえる
- `include/bg431_host.h`、`src/bg431_host.cpp`：MCP25625 の初期化、受信、キープアライブ、コマンドの送信と完了待ち、テレメトリと結果の組み立て
- `src/bg431_jump.cpp`：シリアルモニターからの操作と、`jump` の手順・測定

## 問題が起きた場合

- `INIT FAIL`: VIO / VDDA / GND、JP1、D9 の CS、D7 の RESET、SPI 3本を確認します。
- `LOOPBACK FAIL`: 電源、配線、16 MHz 発振子のボードかを確認します。
- 通常モードで受信できない: 相手も 500 kbps か、CANH / CANL、GND、両端の終端抵抗を確認します。
- `spresense_bg431` でボードと通信できない: ボードの firmware がサンプル点 75% の版か（`can-75pct` 以降）、`stat` の RX overflows / seq gaps、SPI の自己診断の結果を確認します。

## 検証範囲

`spresense` と `spresense_can` の両環境を PlatformIO でビルド確認済みです。

2026-11-01 に USB 接続した SPRESENSE で以下を確認しました（対象コード: `cb5a224`）。

- `/dev/cu.usbserial-1130` から `spresense` 環境の書き込みが成功し、パッケージ検証も成功。
- CS=D9、INT=D8、RST=D7 のコードで MCP25625 の初期化に成功。
- 115200 bps のシリアルログで、ID `0x123` / DLC=8 の内部送受信を確認。先頭バイト `00`～`08` の9回連続で `LOOPBACK PASS` を確認。

この結果は SPI と MCP25625 内部ループバックの確認です。INT の信号動作、外部 CANH / CANL、トランシーバー、相手ノードとの通信は未確認です。実機にはループバック用コードを書き込んであります。

## 参考資料

- [製作者の説明（電源・RESET・終端・16 MHz）](https://asukiaaa.blogspot.com/2021/06/mcp25625-breakout.html)
- [ボードの設計資料](https://github.com/asukiaaa/MCP25625-breakout)
- [Sony SPI 仕様（拡張ボードのピン・JP1）](https://developer.spresense.sony-semicon.com/development-guides/index?lang=en&page=arduino_developer_guide)
- [使用ライブラリ](https://github.com/autowp/arduino-mcp2515/tree/v1.3.1)
