# 文献库说明

本目录保存闭环飞行仿真项目的本地研究资料。新增公开论文均来自作者、会议、期刊或
政府机构的公开页面；`references.bib` 是统一的 UTF-8 元数据源，
`REFERENCES_GBT7714.md` 是按正文引用顺序生成的中文编号格式快照。

## 本地文献与来源

| 编号 | 本地文件 | 来源与校验入口 |
|---|---|---|
| [1] | `基于学习的飞行器智能突防制导方法研究_邱潇颀.pdf` | 用户已有学位论文；PDF 题名页核对 |
| [2] | `大角度机动导弹制导控制系统设计与仿真研究_戴琪昕.pdf` | 用户已有学位论文；PDF 题名页核对 |
| [3] | `导弹制导仿真系统的视景仿真设计_李军.pdf` | 用户已有学位论文；PDF 题名页核对 |
| [4] | `导弹飞行视景仿真系统设计_付沂辰.pdf` | 用户已有学位论文；PDF 题名页核对 |
| [5] | `Penn_Lin_2016_Trick_Simulation_Toolkit.pdf` | [NASA NTRS](https://ntrs.nasa.gov/citations/20150021786) |
| [6] | `Prado_et_al_2013_SIL_Multirotor_UDP.pdf` | [SBMAC Proceedings](https://proceedings.sbmac.org.br/sbmac/article/view/47), DOI: 10.5540/03.2013.001.01.0049 |
| [7] | `Falck_et_al_2021_Dymos.pdf` | [JOSS](https://joss.theoj.org/papers/10.21105/joss.02809), DOI: 10.21105/joss.02809 |
| [8] | `Bonnet_et_al_2022_AirfRANS.pdf` | [NeurIPS Proceedings](https://proceedings.neurips.cc/paper_files/paper/2022/hash/94ab7b23a345f93333eac8748a66c763-Abstract-Datasets_and_Benchmarks.html) |
| [9] | `Marzouk_2025_6DOF_Flight_Dynamics.pdf` | DOI: 10.21833/ijaas.2025.01.004 |
| [10] | `Kamath_et_al_2025_Onboard_Dual_Quaternion_Guidance.pdf` | [arXiv:2508.10439](https://arxiv.org/abs/2508.10439) |
| [11] | `Sharma_et_al_2026_6DOF_Transcription_Failures.pdf` | [arXiv:2605.08420](https://arxiv.org/abs/2605.08420) |

## 完整性校验

所有 11 个 PDF 已通过 `pdfinfo`/`pdftotext` 可读性检查。以下 SHA-256 用于发现下载
截断或文件被意外替换；它不是论文真伪或学术质量证明。

```text
cf969b0e6f7e0ab752ab20dea897cf7597779c03e380c528a75f6c6c581c389a  Bonnet_et_al_2022_AirfRANS.pdf
44ac6e7d925afbc04d0e5cbf12a2b01ed1683bcd9928c3e6210926b61cd87b6b  Falck_et_al_2021_Dymos.pdf
593d0f87c0df94e27d4aa420b58338e1169f6864a9c934fa8883196a4f7fb067  Kamath_et_al_2025_Onboard_Dual_Quaternion_Guidance.pdf
a9be98ef30330c48e5c5f3b663afbc3e46fdde083782e66747e5fbc10030a73c  Marzouk_2025_6DOF_Flight_Dynamics.pdf
6f3031ec653931a19a7abfca7bc63b43181df7d4bd30bec74cd47e620db7a2fb  Penn_Lin_2016_Trick_Simulation_Toolkit.pdf
16e93143950a3f40ca16250de2c5bf7bee59255b20446193a84bed9f78ae1589  Prado_et_al_2013_SIL_Multirotor_UDP.pdf
86fffe37bfcb9f0594874dee0525ac1f5f5effeb5147c014dba60adaff71dcd2  Sharma_et_al_2026_6DOF_Transcription_Failures.pdf
ffd20a66a4dadb0da89c26350f7b3a90056f1b190a2afb619134e9621f946d6f  基于学习的飞行器智能突防制导方法研究_邱潇颀.pdf
0317e55d5c11f67965a988f2ac5a269b286b19d6daa9c843841ebbbea6590759  大角度机动导弹制导控制系统设计与仿真研究_戴琪昕.pdf
74ccdb6775c4d3cc784dd31785cdb0ea7b9cef9edc1813c08c4e5a65c03b6507  导弹制导仿真系统的视景仿真设计_李军.pdf
1896540fd0d424ead9a16adc1c84ba1bdfe273d566870ae47b2e997336a81874  导弹飞行视景仿真系统设计_付沂辰.pdf
```
