Please refer to our website for further details: [https://proroklab.github.io/DVM-SLAM](https://proroklab.github.io/DVM-SLAM)


//OrbSlam3Mono功能

//1,创建并包含orb3对象

//      1,数据结构变化:

//            1)KF和MP新增uuid: 唯一标识KF和MP. 随机生成，基本保证了唯一性(有极低极低概率冲突). 涉及到序列化/反序列化,presave/poseload的mnid标识都替换为uuid

//            2)KF和MP新增creatorAgentId: 标识哪一台Ag创建的

//            3)KeyFrameDatabase新增映射uuidToKeyFrame: 可以从uuid转换到KF*

//

//2,单独开一个线程，只接收实时图像输入，并交给orb3处理: grab_compressed_image/grab_image >> pSLAM->TrackMonocular

//

//3,执行多Ag协同交互任务，包括：

//      1,[本身作为客户端]，创建与其他Ag通信的Peer对象，主动向其他Ag发送最新Kf数据等.(在发送线程中处理)(生成新关键帧时触发)

//            1)向未merge的Ag发送查询Kf的检索描述子: 针对每个Agj,单独记录已发送的KF,然后从Map中收集未发送的KF，打包序列化发送过去.(未作主Ag检查)

//            2)向已merge的Ag发送共享Kf和Mp:       针对每个Agj,单独记录已发送的KF和MP,然后从Map中收集未发送的KF和MP,打包序列化发送过去

//      2,[本身作为服务端]，被动注册数据接收监听，处理其它Ag发送过来的数据.(直接在接收线程处理?)

//            1)接收其它Ag发送的Kf的检索描述子: receiveNewKeyFrameBows (要求本Ag是主Ag)(要求对方Ag未merge)

//                  a)Kf与本机地图描述子检索

//                  b)检索成功的，进一步尝试PNP重定位: 直接在本机上重定位即可.

//                    (本框架是谁Ag的id小，就参考谁的地图，并把小id的Ag的地图发送给大id的Ag去)

//                    大id的Ag作为接收端得到参考地图并尝试回环Merge：receiveMapToAttemptMerge >> LoopClosing::InsertKeyFrame

//                    本机执行merge成功，会将成功的Agid和对应的sim3变换记录在Atlas::successfullyMergedAgentIds

//                    本机执行merge成功，如果对方Agid比自己小，就会刷新本机的参考Agid(Gpid)和对齐变换，并通知本组其它Ag成员一起刷新:OrbSlam3Wrapper::updateSuccessfullyMerged

//            2)接收其他Ag发送的共享Kf和Mp: receiveNewKeyFrames

//                  a)将exKF和exMP反序列化出来，并恢复链接关系，但是先不加入本机地图map

//                  b)将exKF交给LM线程处理:

//                    将地图点加入本机map,将本exKF加入本机map

//                    本exKF和本机map的地图点进行融合

//                    本exKF执行LBA


流程优化：

主要是 多Ag协同交互任务 流程优化:

本框架主要是跑Orb3的流程，然后额外两个任务:1)merge任务：和组外其它Agj尝试重定位merge; 2)共享地图任务：和组内其它Agj进行KF和MP共享，然后各自独立优化

本框架缺陷: 1)merge任务：需要互相传递完整地图，太重量级了;  2)共享地图任务：N二次方复杂度，无法支持较多台Ag共享.



**流程优化:1)merge任务：不传递完整地图，检索KF传给谁就在谁的地图上重定位; 2)共享地图任务：不共享完整KF和MP，只共享轻量KF(id，共连KF，Pose，检索描述子)

**优化后框架

    1，主要跑orb3框架，每个实例由Agid区分，所有KF和MP由uuid唯一标识

    2，merge:[未merge之间] 单组只由主Ag操作,  主Ag接收并缓存所有组员KF检索描述子。

    不同组间全局互发重定位请求，在响应端做重定位，共三个阶段：

    1)描述子检索，成功后分配具体Agi和Agj做重定位。 响应端主Ag执行检索，检索成功后确定实际请求Agi和匹配Agj，然后告知Agi自己主动和Agj去重定位。(优先在主Ag重定位)(主Ag忙碌就转交)

    2)PNP重定位：Agi的请求KF和Agi的响应KF做PNP尝试，成功后即得到对齐变幻Tij，即完成重定位

    3)两个组至少重定位3次即确定merge成功，就基于Tij将大id的组全部对齐到小id的组。(地图要传给主Ag否?)

    3，共享地图:[已merge内部] 组内共享轻量KF(id，共连KF，Pose，检索描述子)

    成员Ag只向主Ag共享完整的KF和MP，然后在主Ag上执行exKF的插入和优化

    主Ag向成员Ag反馈exKF的pose优化结果，成员Ag使用这个信息重新对齐/优化自己地图.(作为组内重定位结果)
