# Optor_exposure_compensation
Updated optor cam driver with exposure compensation.

Replace the ros directory with this new one and recompile the camera.

程序结构：
1, 程序共分为3个模块：1）原代码；2）初始化代码，initialization；3）控制代码，control  

2, 算法过程：程序先进行初始化，这个过程会存入若干张图片（大于2张）和每张图片对应的曝光时间，并调用初始化代码计算出响应函数g。（注意，初始化过程需要保证每张图片的光照相同（即E相同）。之后进入自动控制曝光模式。这个模式下程序会调用图片，曝光时间和计算出来的g自动计算出下一帧的曝光时间 。  

3, 部分表达式含意：
  man_exp——人工曝光值  
  imgnum——用于初始化的图片数量  
  g——响应函数，通过responseRecovery函数得到  
  A，b矩阵，用于计算响应函数g，具体含意见参考文献  
  fit类——用于用多项式拟合响应函数  
  W——权重  
  
4, 已完成部分：实现了控制算法的编写和与原代码的整合，并已通过编译。

5, 目前困难：若运用自带的分辨率，即使在低分辨率模式下，仍会因为A矩阵阶数过大而造成内存溢出。
  
  
