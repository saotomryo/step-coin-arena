# 提出用Docker構成の検証

検証日：2026-10-26。Apple Silicon Mac上のLinux ARM64コンテナで実施しました。[機械可読の結果](verification.json)

|項目|結果|
|---|---|
|Docker Composeビルド|成功。ベース・主要依存・CLIの既定版を固定|
|全単体テスト|273件成功、失敗0、エラー0|
|ROS2・Qt・Genesis CPUスモーク|成功|
|ホストからnoVNC接続|HTTP 200|
|対戦設定・ワークショップ|Docker内の実表示を撮影・確認|
|Genesis描画を伴う短時間対戦|0.3シミュレーション秒、6項目成功|
|120秒の標準対戦|描画なし、ROS2/DDS経由で正常終了|
|120秒試合結果|青8点・赤9点、各6枚、復帰0回|
|Codex CLI|版とdevice-auth引数を確認。認証・実推論は未実施|

120秒の対戦はルールベース構成で確認しました。これをAI戦略の成功や、デモ動画と同じ挙動の証明とは扱いません。デモ動画はMacローカル実行です。

## 再実行

READMEの単体テストとsmokeに加えて、起動済みの仮想デスクトップ上で短時間の対戦検証を実行できます。

```bash
docker compose exec simulator bash -c 'source /opt/ros/jazzy/setup.bash; python tests/verify_match_launcher.py --out /data/verification/gui-short'
```

`docker compose exec` はentrypointを通らないため、ROS2 setupを明示的に読み込みます。READMEの `docker compose run --rm simulator ...` はentrypointで読み込みます。出力先は再実行ごとに新しいディレクトリ名を使ってください。

120秒の描画なし対戦：

```bash
docker compose exec simulator bash -c 'source /opt/ros/jazzy/setup.bash; python scripts/run_competition.py --no-viewer --panel browser --autostart --return-after-match --out /data/verification/full-match'
```

## 整理した旧テスト

インポート時にGPUビューアを起動する旧手動デバッグ用スクリプトを、提出用の単体テスト群から除きました。現在のGenesis表示は上記の実対戦で確認しています。標準時間のテスト期待値も180秒から120秒へ合わせました。

## 未検証

amd64コンテナ、Windows/Linuxホスト、Docker内の実Codex認証・AI推論、Docker内での120秒全編描画、VNCの日本語IME・クリップボード連携は今回の検証対象外です。別環境での動作確認結果はこれらと分けて記録してください。

検証時のデータはGitへ含めていません。リポジトリに認証・個人パス設定・仮想環境・大規模ログを含めない確認も行いました。
