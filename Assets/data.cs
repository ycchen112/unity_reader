using UnityEngine;

[System.Serializable]
public class HandData
{
    public HandPose         leftHandPose;
    public HandPose         rightHandPose;
    public ButtonState      buttons;
    public TriggerState     triggers;
    public ThumbstickState  thumbsticks;
}

[System.Serializable]
public class HandPose
{
    public float[] position;
    public float[] rotation;
    public float[] lineVel;
    public float[] angularVel;
}

[System.Serializable]
public class ButtonState
{
    public int A, B, X, Y;
    public int RTS, LTS;
}

[System.Serializable]
public class TriggerState
{
    public float RI, RH;
    public float LI, LH;
}

[System.Serializable]
public class ThumbstickState
{
    public float[] RT;
    public float[] LT;
}
