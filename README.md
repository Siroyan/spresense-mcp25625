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

## 問題が起きた場合

- `INIT FAIL`: VIO / VDDA / GND、JP1、D9 の CS、D7 の RESET、SPI 3本を確認します。
- `LOOPBACK FAIL`: 電源、配線、16 MHz 発振子のボードかを確認します。
- 通常モードで受信できない: 相手も 500 kbps か、CANH / CANL、GND、両端の終端抵抗を確認します。

## 検証範囲

`spresense` と `spresense_can` の両環境を PlatformIO でビルド確認済みです。実機への書き込み・実機通信は未実施です。

## 参考資料

- [製作者の説明（電源・RESET・終端・16 MHz）](https://asukiaaa.blogspot.com/2021/06/mcp25625-breakout.html)
- [ボードの設計資料](https://github.com/asukiaaa/MCP25625-breakout)
- [Sony SPI 仕様（拡張ボードのピン・JP1）](https://developer.spresense.sony-semicon.com/development-guides/index?lang=en&page=arduino_developer_guide)
- [使用ライブラリ](https://github.com/autowp/arduino-mcp2515/tree/v1.3.1)
