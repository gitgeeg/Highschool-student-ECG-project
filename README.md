高一下学期开学，自己心脏不太舒服
遂有该想法制作便携ecg模块，又想加入呼吸检测，权衡功耗以及便携

所以选择了ads1292r 作为检测ic而esp32进行模块的调试 （集成了呼吸检测 右腿驱动 而且单通道数据方便记录）

注意！本模块的最大作用是验证1292r的检测能力，以及确定外部元件参数
请勿作为其他用途

不过去年春天在花了几周翻了十几遍1292r的数据手册 、参考设计、应用手册、医用心电图教材后
画完板子、采购完器件并且焊接完 要开始编程的时候，发现mcu和1292r的板子始终无法通讯，于是只得暂时放弃。
今年突然心血来潮，换了一个1292r ic ，成功完成初始化通讯。所以花了一个周末完成了该项目的编码工作（plantformio + github copoilt + deepseek ）还得是llm神力啊

嘉立创开源平台：https://oshwhub.com/git-key/ads1292r

代码位于github：https://github.com/gitgeeg/Highschool-student-ECG-project

详情加作者Q群：546926675

<img width="1706" height="1279" alt="微信图片_20260616020918_507_37" src="https://github.com/user-attachments/assets/3beac3c3-528d-47a6-a056-db2919b8b762" />
<img width="1597" height="1027" alt="image" src="https://github.com/user-attachments/assets/70cb98ac-99f7-4c97-a8c4-bff5c971d578" />
