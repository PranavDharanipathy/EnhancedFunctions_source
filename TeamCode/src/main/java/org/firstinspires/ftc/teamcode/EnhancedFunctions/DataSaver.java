package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import android.content.Context;
import android.content.SharedPreferences;
import android.content.SharedPreferences.Editor;

public class DataSaver {

    private SharedPreferences sharedPreferences;
    private Context context;

    public DataSaver(String FILE_NAME, Context context) {

        this.context = context;
        sharedPreferences = this.context.getSharedPreferences(FILE_NAME, Context.MODE_PRIVATE);
    }

    public void saveData(String key, float value) {

        Editor editor = sharedPreferences.edit();
        editor.putFloat(key, value);
        editor.apply();
    }

    public float loadData(String key, float defaultValue) {
        return sharedPreferences.getFloat(key, defaultValue);
    }

    public void remove(String key) { // deletes the key-value pair for the given key in the XML
        sharedPreferences.edit().remove(key).apply();
    }

    public void clearAll() { // clears all data from XML file
        sharedPreferences.edit().clear().apply();
    }

    public void deleteDataFile(String FILE_NAME) { // deletes the XML file
        context.deleteSharedPreferences(FILE_NAME);
    }

}
