package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

public class Debugger {
    private String instanceName;
    private String msgPrefix = "INFO";
    private PrintStream debugLog = null;
    public Debugger(String name)
    {
        instanceName = name;
    }

    public boolean openDebugFile(String logName)
    {
        boolean ret = true;
        try
        {
            debugLog = new PrintStream(new File(Environment.getExternalStorageDirectory() + "/DebugStatements/", logName + ".txt"));
        }
        catch (FileNotFoundException e)
        {
            debugLog = null;
            ret = false;

        }
        return ret;
    }

    public void closeDebugger()
    {
        if(debugLog != null)
        {
            debugLog.close();
            debugLog = null;
        }
    }

    public void debugMessage(String message)
    {
        if(debugLog != null)
        {
            debugLog.print(msgPrefix + "| " + message +"\n");
            debugLog.flush();
        }
    }
    public void setMsgPrefix(String newPrefix)
    {
        msgPrefix = newPrefix;
    }
}
