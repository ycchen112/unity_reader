using UnityEngine;

public static class AndroidLogger
{
    private static readonly AndroidJavaClass logClass = new AndroidJavaClass("android.util.Log");

    public static void Log(string tag, string message)
    {
        logClass.CallStatic<int>("i", tag, message);
    }

    public static void Warn(string tag, string message)
    {
        logClass.CallStatic<int>("w", tag, message);
    }

    public static void Error(string tag, string message)
    {
        logClass.CallStatic<int>("e", tag, message);
    }
}
