# 開発者向けのメモ

***************************

### 未実装機能
 - extra に分類した情報の read/writeの実装
 - write するタイミングの検討について
   - 現在の方法：sub callback でストアしメインループで write
     - [＋] write回数が抑えられる．
       - 各IDへの command が別の topic に乗ってきても，node 側で 1/roop_late [sec] 分の command をまとめてくれる
     - [＋] write の周期が一定以下になり，read の圧迫や負荷の変動が起きづらい
     - [－] 一度 command をストアするので，topic の sub から 最大 1/roop_late [sec] の遅延が生じてしまう．
       - 8ms未満くらいは遅れるが，そもそものtopicの遅延の方が支配的?(topic遅延が6ms，callback->writeが遅延2ms)
   - もう一つの方法：sub callback で直接 write
     - [＋] callback後の遅延は生じない
     - [－] topic の pub の仕方によってはwrite回数が増えてしまう
       - 例えば，ID:5へ指令する command topic と ID:6が別のノードからpubされているとすると，callbackは2回呼ばれる．一度ストアしてからまとめてWrite方式だとwriteは1回だが，callbackで直接Write方式だとwriteも2回


### Node終了時の停止について

- bus_watchbdogで停止できそう？
  - RAM値なので簡単に書き込みできる
  - 初期化時の最初に書き込まないと，bus_watchbdogがエラー中のサーボのgoal値を書き換えられない
  - Velocity・current制御モードの場合だけ停止するようにしていたが，全部停止するようにしてもいいかもしれない．
- bus_watchbdogのバグあり．
  - XC330は停止しない？
  - XM540は停止はするけどhomming_offsetの値分動いてから停止する．なんで？？？ (Firmware upate 46 to 48 で治った？治ってないやんけ！)


1. 通信断絶時の停止について
   - 速度・電流・PWM制御モードではデフォルトでbus_watchdogによって止まるようにしたい．
   - `term/servo_auto_stop`パラメータで制御する？
   - 取りあえずbus_watchdogでだいたいは大丈夫なはず．
     - XM540の位置制御系では，homing_offsetが設定時のバグが残っているが，速度・電流・PWM制御系では問題ない．
   - (例外) XC330はそもそも止まらない．
     - こいつのみ別途止める必要がある？ないか．
     - まあ，通信断絶
  
2. ノードの明示的な停止について
    - こいつも `term/servo_auto_stop` で制御する．
    - 速度・電流・PWM制御モードについてはbus_watchbdogでクリアしている．
      - 念のため即時停止するように最も短いbus_watchdogを再設定するか？
    - 位置制御系については現在位置を指令しなおすしか手がないかな．

3. トルクの瞬間的なオンオフでは止まらなそう．
   - XC330は止まらずに最初の目標値目指して回り続ける (goal値はオンオフされた瞬間のものに更新されてる)
   - XM540は止まるときと，止まらずに加速するときがある．わけわからん．
  
### 遭遇したバグ

Fast Sync Read で 1~6 のサーボに read を送った状況で，ID:1モータが断線していたとする．
当然，Fast Sync Read の return packet であるひとつながりのパケットは帰ってこないのでタイムアウト．
その直後，1~6の各モータに対してPingを送るとなぜか**全て**のモータがタイムアウトする．
「Fast Sync Read + 1つ目のIDの断線によりタイムアウト + 左記の直後に Ping」という状況だと，断線してないモータもタイムアウトしてしまうという問題が起きる．
　- Sync Read だとこの問題は起きない．
　- Fast Sync Read で指定するIDの先頭であるID:1 **以外** のモータを断線させても問題は起きない．
　- Fast Sync Read で指定するIDの順番を逆順にしたうえで，ID:1を断線させても問題は起きない．
　- Fast Sync Read で指定するIDの順番を逆順にしたうえで，ID:6を断線させると問題が起きる．

