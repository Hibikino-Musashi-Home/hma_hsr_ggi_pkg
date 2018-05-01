#!/usr/bin/env python
# -*- coding: utf-8 -*-




#==================================================

## @file
## @author Yutaro ISHIDA
## @brief Go Get ItのステートマシンROSノード

#==================================================




import sys
import roslib


sys.path.append(roslib.packages.get_pkg_dir("hma_common_pkg") + "/script/import/common")
from common_import import *
from common_global import *
from common_param import *


sys.path.append(roslib.packages.get_pkg_dir("hma_lib_pkg") + "/script/src/lib")
from libutil import *
from libsound import *
from libspeech import *


if GP_ROBOT != "none":
    sys.path.append(roslib.packages.get_pkg_dir("hma_" + GP_ROBOT + "_pkg") + "/script/import/robot")
    from robot_import import *
    from robot_global import *
    from robot_param import *

    sys.path.append(roslib.packages.get_pkg_dir("hma_" + GP_ROBOT + "_pkg") + "/script/src/lib")
    from libhsr import *




#==================================================

# グローバル

#==================================================




#==================================================

## @class 初期化ステート

#==================================================
class Init(
    smach.State
):
    #==================================================
    
    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self
    ):
        smach.State.__init__(self, outcomes = ["normal", "except"])


        return


    #==================================================
    
    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):


        return "normal"




#==================================================

## @class スタート待ちステート

#==================================================
class WaitStart(
    smach.State
):
    #==================================================
    
    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libsound,
        libhsr
    ):
        smach.State.__init__(self, outcomes = ["normal", "except"])

        self._libsound = libsound
        self._libhsr = libhsr


        return


    #==================================================
    
    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):
        self._libsound.soundEffectSync(0)
        self._libhsr.nonDefaultTTSSay("I'm preparing to start.", GP_LANG)
        self._libhsr.wholeBodyMoveToGo()
        self._libhsr.gripperCommand(1.2)

        self._libsound.soundEffectSync(0)
        self._libhsr.nonDefaultTTSSay("Push my hand to start.", GP_LANG)

        while not rospy.is_shutdown():
            if self._libhsr.armSwitch():
                break

        self._libhsr.nonDefaultTTSSay("Let's go!", GP_LANG)


        return "normal"




#==================================================

## @class 標準スタートステート

#==================================================
class NormalStart(
    smach.State
):
    #==================================================
    
    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
    ):
        smach.State.__init__(self, outcomes = ["normal", "except"])


        return


    #==================================================
    
    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):


        return "normal"




#==================================================

## @class オペレータを追跡するステート

#==================================================
class FollowOperator(
    smach.State
):
    #==================================================
    
    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libspeech,
        libhsr
    ):
        smach.State.__init__(self, outcomes = ["normal", "except"])

        self._libspeech = libspeech
        self._libhsr = libhsr


        return


    #==================================================
    
    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):
        self._libhsr.talkRequest("Please wait for the following.", GP_LANG)

        Popen(["rosrun", "hma_common_pkg", "follow_operator_node"])
        Popen(["rosrun", "hma_hsr_pkg", "follow_target_node.py"])
        rospy.sleep(5.0)

        while not rospy.is_shutdown():
            try:
                txt_and_score = self._libspeech.recSpeech(
                    "",
                    "Please say stop.",
                    5.0,
                    10.0,
                    7.5,
                )
            except:
                continue

            if len(re.findall("\sstop\s", " " + txt_and_score[0]["TXT"] + " ")) != 0:
                self._libhsr.talkRequest("Okay.", GP_LANG)
                self._libhsr.omniBaseVel(0.0, 0.0, 0.0)
                self._libhsr.wholeBodyMoveToGo()
                call(["rosnode", "kill", "/follow_operator_node"])
                call(["rosnode", "kill", "/follow_target_node"])
                break


        return "normal"




#==================================================

## @class 学習するステート

#==================================================
class Train(
    smach.State
):
    #==================================================
    
    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libspeech,
        libhsr,
        cmu_phoneme
    ):
        smach.State.__init__(self, outcomes = ["follow", "solve", "except"])

        self._libspeech = libspeech
        self._libhsr = libhsr
        self._cmu_phoneme = cmu_phoneme

        self._obj_param = []

        self._phoneme = []
        self._keyword = []

        self._place_x = []
        self._place_y = []
        self._place_yaw = []


        return


    #==================================================
    
    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):
        x, y, yaw = self._libhsr.omniBasePose()
        self._place_x.append(x)
        self._place_y.append(y)
        self._place_yaw.append(yaw)


        rospy.set_param("/get_yes_or_no_node/txt", "Do you wanna terminate training?")
        rospy.set_param("/get_yes_or_no_node/speaker_time", 0.0)
        rospy.set_param("/get_yes_or_no_node/speaker_timeout", 0.0)
        rospy.set_param("/get_yes_or_no_node/rec_timeout", 5.0)


        if call(["rosrun", "hma_nlp_pkg", "get_yes_or_no_node.py"]) == 0:
            self._libhsr.nonDefaultTTSSay("Okay. Terminate training.", GP_LANG)

            rospy.set_param(rospy.get_name() + "/obj_param", self._obj_param)
            rospy.set_param(rospy.get_name() + "/phoneme", self._phoneme)
            rospy.set_param(rospy.get_name() + "/keyword", self._keyword)
            rospy.set_param(rospy.get_name() + "/place/amount", len(self._place_x))
            rospy.set_param(rospy.get_name() + "/place/x", self._place_x)
            rospy.set_param(rospy.get_name() + "/place/y", self._place_y)
            rospy.set_param(rospy.get_name() + "/place/yaw", self._place_yaw)


            return "solve"


        self._libhsr.nonDefaultTTSSay("Okay. Continue training.", GP_LANG)


        self._obj_param.append([])


        get_obj_param = GetObjParam(self._libhsr)
        for i in xrange(1):
            weight, size, strain = get_obj_param.getObjParamByOperator()
            self._obj_param[-1].append([weight, size, strain])


        self._phoneme.append([])
        self._keyword.append([])

        while not rospy.is_shutdown():
            rospy.sleep(1.0)

            try:
                txt_and_score = self._libspeech.recSpeech(
                    "Please say keyword.",
                    "",
                    0.0,
                    0.0,
                    5.0,
                )
            except:
                continue

            for i in xrange(len(txt_and_score)):
                if len(re.findall("\sterminate\s", " " + txt_and_score[i]["TXT"] + " ")) != 0:
                    self._libhsr.nonDefaultTTSSay("Thank you.", GP_LANG)
                    return "follow"

            for i in xrange(len(txt_and_score)):
                txt = txt_and_score[i]["TXT"]
                txt = txt.split(" ")

                for j in xrange(len(txt)):
                    try:
                        self._phoneme[-1].append(" ".join(self._cmu_phoneme[str(txt[i])][0]))
                        self._keyword[-1].append(txt[j])
                    except:
                        continue




#==================================================

## @class Go Get Itのコマンドを解くステート

#==================================================
class SolveGGICmd(
    smach.State
):
    #==================================================
    
    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libspeech,
        libhsr,
        cmu_phoneme
    ):
        smach.State.__init__(self, outcomes = ["normal", "except"])

        self._libspeech = libspeech
        self._libhsr = libhsr
        self._cmu_phoneme = cmu_phoneme


        return


    #==================================================
    
    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):
        obj_param = rospy.get_param(rospy.get_name() + "/obj_param", [])
        phoneme = rospy.get_param(rospy.get_name() + "/phoneme", [])
        keyword = rospy.get_param(rospy.get_name() + "/keyword", [])
        #最後のインデックスは学習終了地点
        place_amount = rospy.get_param(rospy.get_name() + "/place/amount", 0)
        place_x = rospy.get_param(rospy.get_name() + "/place/x", [])
        place_y = rospy.get_param(rospy.get_name() + "/place/y", [])
        place_yaw = rospy.get_param(rospy.get_name() + "/place/yaw", [])


        while not rospy.is_shutdown():
            try:
                txt_and_score = self._libspeech.recSpeech(
                    "Please say command.",
                    "",
                    0.0,
                    0.0,
                    5.0,
                )
            except:
                continue


            prob = [0] * (place_amount - 1)

            for i in xrange(len(txt_and_score)):
                txt = txt_and_score[i]["TXT"]
                txt = txt.split(" ")

                for j in xrange(len(txt)):
                    distance_min = 1000
                    index = 0

                    for k in xrange(int(place_amount - 1)):
                        for l in xrange(len(phoneme[k])):
                            try:
                                if Levenshtein.distance(str(" ".join(self._cmu_phoneme["bring"][0])), str(" ".join(self._cmu_phoneme[str(txt[j])][0]))) < 2:
                                    continue
                                if Levenshtein.distance(str(" ".join(self._cmu_phoneme["me"][0])), str(" ".join(self._cmu_phoneme[str(txt[j])][0]))) < 2:
                                    continue
                                distance = Levenshtein.distance(str(phoneme[k][l]), str(" ".join(self._cmu_phoneme[str(txt[j])][0])))
                            except:
                                continue

                            rospy.loginfo("[" + rospy.get_name() + "]: Distance: " + str(distance))

                            if distance_min > distance:
                                distance_min = distance
                                index = k

                    if distance_min <= 3:
                        prob[index] += 1

            rospy.loginfo("[" + rospy.get_name() + "]: Probability: ")
            pprint.pprint(prob)


            index = prob.index(max(prob))

            if prob[index] == 0:
                self._libhsr.nonDefaultTTSSay("Sorry, I can't understand." + str(index) + ".", GP_LANG)
                continue


            rospy.set_param("/get_yes_or_no_node/txt", "Okay. Should I go to place " + str(index) + "?")
            rospy.set_param("/get_yes_or_no_node/speaker_time", 0.0)
            rospy.set_param("/get_yes_or_no_node/speaker_timeout", 0.0)
            rospy.set_param("/get_yes_or_no_node/rec_timeout", 5.0)

            if call(["rosrun", "hma_nlp_pkg", "get_yes_or_no_node.py"]) == 1:
                continue


            self._libhsr.nonDefaultTTSSay("Okay. Please move out of my way.", GP_LANG)

            rospy.set_param("/go_place_node/target/x", place_x[index])
            rospy.set_param("/go_place_node/target/y", place_y[index])
            rospy.set_param("/go_place_node/target/yaw", place_yaw[index])

            call(["rosrun", "hma_hsr_pkg", "go_place_node.py"])


            self._libhsr.wholeBodyMoveToHeadPositions(-90.0 * G_DEG2RAD, -15.0 * G_DEG2RAD)
            rospy.sleep(5.0)

            #get_obj_param = GetObjParam(self._libhsr)
            #get_obj_param.getForcePre()
            #把持動作
            #get_obj_param.getForcePost()
            #weight, size, strain = get_obj_param.getObjParamByOwn()
            #
            #for i in xrange(len(obj_param[index])):
            #    if math.sqrt(math.pow(weight - obj_param[index][i][0], 2) + math.pow(size - obj_param[index][i][1], 2) + math.pow(strain - obj_param[index][i][2], 2)) < 30.0:
            #        self._libhsr.nonDefaultTTSSay("This is correct object.", GP_LANG)
            #        break   
            #else:
            #    self._libhsr.nonDefaultTTSSay("This is incorrect object.", GP_LANG)


            self._libhsr.nonDefaultTTSSay("I can't find. I go back.", GP_LANG)

            rospy.set_param("/go_place_node/target/x", place_x[-1])
            rospy.set_param("/go_place_node/target/y", place_y[-1])
            rospy.set_param("/go_place_node/target/yaw", place_yaw[-1])

            call(["rosrun", "hma_hsr_pkg", "go_place_node.py"])


        return "normal"




#==================================================

## @class 正常終了ステート

#==================================================
class NormalEnd(
    smach.State
):
    #==================================================
    
    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self
    ):
        smach.State.__init__(self, outcomes = ["normal"])


        return


    #==================================================
    
    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):


        return "normal"




#==================================================

## @class 例外終了ステート

#==================================================
class Except(
    smach.State
):
    #==================================================
    
    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self
    ):
        smach.State.__init__(self, outcomes = ["except"])


        return


    #==================================================
    
    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return


    #==================================================

    ## @fn 実行関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def execute(
        self,
        userdata
    ):


        return "except"




#==================================================

## @class 物体パラメータ取得クラス

#==================================================
class GetObjParam:
    #==================================================
    
    ## @fn コンストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __init__(
        self,
        libhsr
    ):
        self._libhsr = libhsr

        self._force_pre = 0.0
        self._force_post = 0.0

        self._torque_low = -0.01
        self._torque_high = -0.05


        return




    #==================================================
    
    ## @fn デストラクタ
    ## @brief
    ## @param
    ## @return

    #==================================================
    def __del__(
        self
    ):


        return




    #==================================================

    ## @fn 把持前重力設定関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def getForcePre(
        self
    ):
        try:
            self._force_pre = self._libhsr.getObjForce()
        except:
            raise HMAExWarn()


        return




    #==================================================

    ## @fn 把持後重力設定関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def getForcePost(
        self
    ):
        try:
            self._force_post = self._libhsr.getObjForce()
        except:
            raise HMAExWarn()


        return




    #==================================================

    ## @fn ユーザと物体パラメータを取得する関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def getObjParamByOperator(
        self
    ):
        self.getForcePre()

        self._libhsr.nonDefaultTTSSay("Give me the object.", GP_LANG)
        rospy.sleep(1.0)

        self._libhsr.gripperGrasp(self._torque_low)
        rospy.sleep(1.0)

        self.getForcePost()

        weight, size, strain = self.getObjParam()

        self._libhsr.nonDefaultTTSSay("I release the object.", GP_LANG)
        rospy.sleep(1.0)

        self._libhsr.gripperCommand(1.2)
        rospy.sleep(1.0)


        return weight, size, strain




    #==================================================

    ## @fn 自身で物体パラメータを取得する関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def getObjParamByOwn(
        self
    ):
        weight, size, strain = self.getObjParam()


        return weight, size, strain




    #==================================================

    ## @fn 物体パラメータを取得する関数
    ## @brief
    ## @param
    ## @return

    #==================================================
    def getObjParam(
        self
    ):
        self._libhsr.gripperGrasp(self._torque_low)
        rospy.sleep(1.0)

        try:
            position_pre = self._libhsr.armHandMotorPosition()
        except:
            raise HMAExWarn()

        self._libhsr.gripperGrasp(self._torque_high)
        rospy.sleep(1.0)

        try:
            position_post = self._libhsr.armHandMotorPosition()
        except:
            raise HMAExWarn()

        force_diff = math.sqrt(sum([math.pow(post - pre, 2) for (pre, post) in zip(self._force_pre, self._force_post)]))
        position_diff = position_post - position_pre
        
        return force_diff / 9.81 * 1000, position_pre, position_diff / abs(self._torque_high - self._torque_low) * -1  #重量[g]，サイズ[rad]，変形率[rad/トルク]




#==================================================

# メイン

#==================================================
if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])


    libutil = LibUtil()
    libsound = LibSound()
    libspeech = LibSpeech()
    libhsr = LibHSR()

    cmu_phoneme = cmudict.dict()


    rospy.loginfo("[" + rospy.get_name() + "]: Task start")
    rospy.loginfo("[" + rospy.get_name() + "]: Please input first state name")


    start_state = raw_input(">>> ")
    if not start_state:
        start_state = "NormalStart"


    #==================================================
    
    # ステートマシンの宣言

    #==================================================
    ssm = smach.StateMachine(outcomes = ["exit"])

    with ssm:
        smach.StateMachine.add(
            "Init",
            Init(), 
            transitions = {
                "normal":"WaitStart",
                "except":"Except"
            }
        )
        smach.StateMachine.add(
            "WaitStart",
            WaitStart(libsound, libhsr),
            transitions = {
                "normal":start_state,
                "except":"Except"
            }
        )
        smach.StateMachine.add(
            "NormalStart",
            NormalStart(),
            transitions = {
                "normal":"FollowOperator",
                "except":"Except"
            }
        )
        smach.StateMachine.add(
            "FollowOperator",
            FollowOperator(libspeech, libhsr),
            transitions = {
                "normal":"Train",
                "except":"Except"
            }
        )
        smach.StateMachine.add(
            "Train",
            Train(libspeech, libhsr, cmu_phoneme),
            transitions = {
                "follow":"FollowOperator",
                "solve":"SolveGGICmd",
                "except":"Except"
            }
        )
        smach.StateMachine.add(
            "SolveGGICmd",
            SolveGGICmd(libspeech, libhsr, cmu_phoneme),
            transitions = {
                "normal":"NormalEnd",
                "except":"Except"
            }
        )
        smach.StateMachine.add(
            "NormalEnd",
            NormalEnd(),
            transitions = {
                "normal":"exit"
            }
        )
        smach.StateMachine.add(
            "Except",
            Except(),
            transitions = {
                "except":"exit"
            }
        )

    sris = smach_ros.IntrospectionServer("ssm", ssm, "/SM_ROOT")
    sris.start()


    while not rospy.is_shutdown():
        ssm.execute()
