# Stackchan_Mic

m5stack-avatar-micにサーボの機能とLED表示機能を追加しました。

## 機能
- マイク入力によるリップシンク（口の動き）
- サーボモーターによる頭の動き制御
- M5GoBottomのLED表示機能
- 複数のM5Stackデバイスに対応（Core, Core2/AWS, CoreS3/SEなど）
- 複数のサーボタイプに対応（SG90 PWM, Feetech SCS0009, Dynamixel XL330）
- TakaoBaseによる電源供給オプション

## ビルドの注意事項
CoreS3/SEだと、FastLEDのライブラリがplatformio.iniのlib_depsに入っているとビルドエラーになるため、FastLEDとUSE_LEDをコメントアウトしてあります。
Core1/Core2でLEDを利用したい場合は、platformio.ini(30行目付近)とmain.cpp(41行目付近)のコメント部分のコメントアウトを外してください。


## セットアップ

1. [data/yaml](./data/yaml/)内のファイル（SC_BasicConfig.yaml）を、microSDカードのyamlフォルダにコピーしてください。
2. SC_BasicConfig.yamlを編集して、使用するハードウェアに合わせて設定してください。
   - サーボのピン設定
   - サーボのオフセット
   - サーボの中心位置と可動範囲
   - TakaoBaseの使用有無
   - サーボタイプ（PWM, SCS, DYN_XL330, RT_DYN_XL330）

## 操作方法

- ボタンA: カラーパレットの変更
- ボタンA（ダブルクリック）: LEDのON/OFF切り替え
- ボタンB: 動作モードの切り替え（0:常時動作、1:音声検出時のみ動作、2:回転あり(SCS0009のみ)）
- ボタンC: ModuleLLMにSerialで”A"を出力する。
- ボタンC(ダブルクリック): 外部出力のON/OFF切り替え
- 電源ボタン（クリック）: 再起動

## 依存ライブラリ

- [M5Stack-Avatar](https://github.com/stack-chan/M5Stack-Avatar)
- [stackchan-arduino](https://github.com/stack-chan/stackchan-arduino)
- FastLED（M5GoBottomのLED制御用）

## ハードウェア要件

- M5Stack系デバイス（Core, Core2, CoreS3, M5StickC, M5StickC-Plus2, M5AtomS3など）
- サーボモーター（SG90, SCS0009, Dynamixel XL330など）
- microSDカード（設定ファイル用）
- オプション: [Stack-chan_Takao_Base](https://ssci.to/8905)（電源供給用）
