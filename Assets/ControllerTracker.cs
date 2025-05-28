using UnityEngine;
using System.Collections;
using UnityEngine.XR;

public class ControllerTracker : MonoBehaviour
{
    // 记录标定坐标系的变换矩阵
    private Matrix4x4 calibrationMatrix;
    private bool isCalibrated = false;

    public OVRCameraRig cameraRig;

    private float abHoldTime = 0f;
    private bool isHapticTriggered = false;

    IEnumerator TriggerHapticPulse()
    {
        float freq = 1.0f;
        float amp = 0.5f;

        OVRInput.SetControllerVibration(freq, amp, OVRInput.Controller.RTouch);
        yield return new WaitForSeconds(0.5f);  // 震动0.5秒
        OVRInput.SetControllerVibration(0, 0, OVRInput.Controller.RTouch);
    }

    // Start is called before the first frame update
    void Start()
    {
        //// 头显位置
        //Vector3     headPosition = Camera.main.transform.position;
        //Quaternion  headRotation = Camera.main.transform.rotation;

    }

    // Update is called once per frame
    void Update()
    {
        var active = OVRInput.GetConnectedControllers();
        Debug.Log($"[OVR] Connected Controllers: {active}");

        // 获取trackingSpace Transform
        // TODO：位置输出现在相对于头显，修改成trackingSpace方法可能能解决

        // 获取左右手位置
        Vector3    rightHandLocalPosition = OVRInput.GetLocalControllerPosition(OVRInput.Controller.LTouch);
        Vector3    leftHandLocalPosition  = OVRInput.GetLocalControllerPosition(OVRInput.Controller.RTouch);
        Quaternion leftHandLocalRotation  = OVRInput.GetLocalControllerRotation(OVRInput.Controller.LTouch);
        Quaternion rightHandLocalRotation = OVRInput.GetLocalControllerRotation(OVRInput.Controller.RTouch);

        // 获取左右手在世界坐标系的位置
        Vector3 rightHandWorldPosition =
            Camera.main.transform.position +
            Camera.main.transform.rotation * rightHandLocalPosition;
        Quaternion rightHandWorldRotation = Camera.main.transform.rotation * rightHandLocalRotation;
        Matrix4x4 rightHandWorldTrans = Matrix4x4.TRS(rightHandWorldPosition, rightHandWorldRotation, Vector3.one);

        Vector3 leftHandWorldPosition =
            Camera.main.transform.position +
            Camera.main.transform.rotation * leftHandLocalPosition;
        Quaternion leftHandWorldRotation = Camera.main.transform.rotation * leftHandLocalRotation;
        Matrix4x4 leftHandWorldTrans = Matrix4x4.TRS(leftHandWorldPosition, leftHandWorldRotation, Vector3.one);

        // a+b同时按下记录位置
        bool aPressed = OVRInput.Get(OVRInput.Button.One, OVRInput.Controller.RTouch);
        bool bPressed = OVRInput.Get(OVRInput.Button.Two, OVRInput.Controller.RTouch);
        if (aPressed && bPressed)
        {
            abHoldTime += Time.deltaTime;
            Calibrate(rightHandWorldTrans);
            if (abHoldTime >= 1.0f && !isHapticTriggered)
            {
                StartCoroutine(TriggerHapticPulse());
                isHapticTriggered = true; // 避免重复触发
            }
            else
            {
                abHoldTime = 0f;
                isHapticTriggered = false;
            }
        }

        // 如果标定了位置，就计算标定坐标系下的位置
        Matrix4x4 newRightPoseTrans = Matrix4x4.identity;
        Matrix4x4 newLeftPoseTrans = Matrix4x4.identity;
        if (isCalibrated)
        {
            newRightPoseTrans = calibrationMatrix.inverse * rightHandWorldTrans;
            newLeftPoseTrans = calibrationMatrix.inverse * leftHandWorldTrans;
        }
        else
        {
            newRightPoseTrans = rightHandWorldTrans;
            newLeftPoseTrans = leftHandWorldTrans;
        }

        // 获取速度
        Vector3 rightHandLineVel = OVRInput.GetLocalControllerVelocity(OVRInput.Controller.RTouch);
        Vector3 leftHandLineVel  = OVRInput.GetLocalControllerVelocity(OVRInput.Controller.LTouch);
        Vector3 rightAngularVel  = OVRInput.GetLocalControllerAngularVelocity(OVRInput.Controller.RTouch);
        Vector3 leftAngularVel   = OVRInput.GetLocalControllerAngularVelocity(OVRInput.Controller.LTouch);

        string strRightVel  = $"{rightHandLineVel.x:F6} {rightHandLineVel.y:F6} {rightHandLineVel.z:F6} " +
            $"{rightAngularVel.x:F6} {rightAngularVel.y:F6} {rightAngularVel.z:F6}";
        string strLeftVel   = $"{leftHandLineVel.x:F6} {leftHandLineVel.y:F6} {leftHandLineVel.z:F6} " +
            $"{leftAngularVel.x:F6} {leftAngularVel.y:F6} {leftAngularVel.z:F6}";
        string strVel = strLeftVel + "|" + strRightVel;
        string strLogVelFlag = "allHandVel";

        AndroidLogger.Log(strLogVelFlag, strVel);

        // 检查按键
        bool aButton = OVRInput.Get(OVRInput.Button.One, OVRInput.Controller.RTouch);
        bool bButton = OVRInput.Get(OVRInput.Button.Two, OVRInput.Controller.RTouch);
        bool thumbstickButton = OVRInput.Get(OVRInput.Button.PrimaryThumbstick, OVRInput.Controller.RTouch);

        bool xButton = OVRInput.Get(OVRInput.Button.One, OVRInput.Controller.LTouch);
        bool yButton = OVRInput.Get(OVRInput.Button.Two, OVRInput.Controller.LTouch);
        bool leftThumbstickButton = OVRInput.Get(OVRInput.Button.PrimaryThumbstick, OVRInput.Controller.LTouch);

        // 读取扳机（Trigger）按下程度（0 ~ 1）
        float rightIndexTrigger = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch);
        float rightHandTrigger  = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.RTouch);
        float leftIndexTrigger  = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
        float leftHandTrigger   = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.LTouch);

        // 读取摇杆（Thumbstick）位置（二维向量）
        Vector2 rightThumbstick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch);
        Vector2 leftThumbstick  = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch);

        HandData data = new HandData();
        data.rightHandPose = new HandPose
        {
            //position    = new float[] { newRightPoseTrans.GetColumn(3).x, newRightPoseTrans.GetColumn(3).y, newRightPoseTrans.GetColumn(3).z },
            position    = new float[] { rightHandWorldPosition.x, rightHandWorldPosition.y, rightHandWorldPosition.z },
            rotation    = new float[] { rightHandWorldRotation.x, rightHandWorldRotation.y, rightHandWorldRotation.z, rightHandWorldRotation.w },
            lineVel     = new float[] { rightHandLineVel.x, rightHandLineVel.y, rightHandLineVel.z },
            angularVel  = new float[] { rightAngularVel.x, rightAngularVel.y, rightAngularVel.z }
        };
        data.leftHandPose = new HandPose
        {
            position    = new float[] { leftHandWorldPosition.x, leftHandWorldPosition.y, leftHandWorldPosition.z },
            rotation    = new float[] { leftHandWorldRotation.x, leftHandWorldRotation.y, leftHandWorldRotation.z, leftHandWorldRotation.w },
            lineVel     = new float[] { leftHandLineVel.x, leftHandLineVel.y, leftHandLineVel.z },
            angularVel  = new float[] { leftAngularVel.x, leftAngularVel.y, leftAngularVel.z }
        };
        data.buttons = new ButtonState
        {
            A   = aButton ? 1 : 0,
            B   = bButton ? 1 : 0,
            X   = xButton ? 1 : 0,
            Y   = yButton ? 1 : 0,
            RTS = thumbstickButton ? 1 : 0,
            LTS = leftThumbstickButton ? 1 : 0
        };
        data.triggers = new TriggerState
        {
            RI = rightIndexTrigger,
            RH = rightHandTrigger,
            LI = leftIndexTrigger,
            LH = leftHandTrigger
        };
        data.thumbsticks = new ThumbstickState
        {
            RT = new float[] { rightThumbstick.x, rightThumbstick.y },
            LT = new float[] { leftThumbstick.x, leftThumbstick.y }
        };

        string data_json = JsonUtility.ToJson(data);
        AndroidLogger.Log("handDataJson", data_json);

    }

    private void Calibrate(Matrix4x4 rightHandWorldTrans)
    {
        // 记录当前右手手柄的位置和旋转作为新的坐标原点
        // 在世界坐标系下
        calibrationMatrix = rightHandWorldTrans;

        isCalibrated = true;
        // Debug.Log("[HandTracker] Calibration set to Right Hand!");
    }
}
