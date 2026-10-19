# Autonomous Driving Core — 全新獨立實作

本資料夾從空白 Python／LaTeX 檔案建立。模型與內容只依原始題目、原始提交模板、Lecture 4／5／6；沒有匯入舊解答模組或使用舊控制增益。來源檔案的 SHA-256 記錄於 `source_checksums.json`。

## 執行

```bash
python -m pip install -r requirements.txt
python run.py
pdflatex main.tex
pdflatex main.tex
```

可從任意工作目錄執行 Python，輸出位於本資料夾 `figures/` 與 `data/`。編譯 LaTeX 時請進入本資料夾。Overleaf 直接上傳 ZIP，將 Main document 設定為 `main.tex`，修改作者姓名即可。

## 檔案

- `bicycle.py`：新寫的參數資料類別、題目式 (10) 矩陣、LQR 與 Pepy 微分方程。
- `kinematics.py`：自行實作 RK4；定轉角、正弦與方波模擬及解析解／步長驗證。
- `lane_change.py`：獨立的矩陣指數主模擬、分段 ODE 驗證、收斂／轉向速率檢查及全域幾何重建。
- `design.json`：本次新搜尋得到的 Q、R；K 在每次執行時計算，不以既有 K 代入。
- `search/design_lqr.py`：可重現新權重搜尋，固定 seed=642。
- `search/design_discrete.py`：額外的離散極點搜尋，嘗試將 X=100 的條件一併納入。
- `data/lane_validation.json`：完整數值結果，以及分開列出的 `course_model_checks`、`literal_checks`。
- `main.tex`、`main.pdf`：沿原模板順序寫成的全新報告；1.1 assumptions 使用 (1)、(2)…，全文不縮排。

選擇性重新搜尋（正常重跑不需要）：

```bash
OPENBLAS_NUM_THREADS=1 python search/design_lqr.py
OPENBLAS_NUM_THREADS=1 python search/design_discrete.py
```

搜尋產生新 JSON，但不會自動覆蓋主程式的 `design.json`；這避免搜尋實驗默默改變報告對應的設計。

## 新實作的驗證結果

| 題目式 (10) 的檢查 | 結果 | 門檻 |
|---|---:|---:|
| 最大 e2 絕對值 | 0.00952359 rad | 0.01 rad |
| 兩次轉換收斂時間 | 0.99733 s | 1 s |
| 最大連續轉向速率 | 23.80862 rad/s | 25 rad/s |
| 矩陣指數 vs. 獨立 ODE | 1.71e-12 | 1e-8（程式核對門檻） |

輸出間隔 0.01 秒，另以 0.0001 秒網格稽核；收斂同時檢查 e1 與 e2，並要求持續到下一次轉換或模擬結束。全域重建的反算檢查約 4.52e-16 m，是座標轉換驗證，不是另一套車輛物理驗證。

## 仍未通過的條件：請勿把本版本當成全部通過

新程式在名義 X_des=100 m 時得到約 (100.000307, 0.086239) m，航向 −0.003557 rad。因此尚未達到字面上的指定終點與零航向。兩組離散極點搜尋也沒有找到滿足所測試全部限制的候選；有限搜尋失敗不是全域無解證明。

參考 yaw-rate 以一個 0.01 秒寬脈衝表示航向轉換，這是對尖角的有限時間近似。式 (10) 的 e2 數值通過，但如果改為比較理想折線瞬間跳變的切線，最大航向誤差約 0.055499 rad，沒有通過 0.01 rad。

這份是新寫、可重跑、明確記錄通過與未通過條件的 Core 候選解。沒有調大題目誤差門檻、隱藏失敗旗標、裁掉轉折瞬間，或強制把終點設為零。要主張完全符合，仍需解決參考轉折及终點解讀與模型的相容性。
