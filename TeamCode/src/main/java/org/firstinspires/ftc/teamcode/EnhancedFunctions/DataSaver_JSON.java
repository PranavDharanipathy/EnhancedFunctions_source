package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.io.BufferedWriter;
import java.io.BufferedReader;
import java.io.FileWriter;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

public class DataSaver_JSON {

    private final Gson gson = new GsonBuilder().setPrettyPrinting().create();

    private BufferedWriter dataFile;
    private BufferedReader dataReader;

    public DataSaver_JSON(String fileName) {

        try {
            dataFile = new BufferedWriter(new FileWriter(fileName));
            dataReader = new BufferedReader(new FileReader(fileName));

        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    /// Adds variable to JSON
    public synchronized void addData(Object data, boolean nextLine) {

        String nl = nextLine ? "\n" : "";

        String currentData = gson.fromJson(dataReader, String.class);

        try {
            dataFile.write(currentData + gson.toJson(data) + nl);
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    /// Adds array to JSON
    public synchronized void addData(Object[] data, boolean nextLine) {

        String nl = nextLine ? "\n" : "";

        String currentData = gson.fromJson(dataReader, String.class);

        try {
            dataFile.write(currentData + gson.toJson(data) + nl);
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    /// Reads data from JSON
    public String[] retrieveData() {

        String[] data = gson.fromJson(dataReader, String[].class);

        //want a list/array type value
        ArrayList<String> allData = new ArrayList<>(Arrays.asList(data));
        return allData.toArray(new String[0]);
    }


    public void close() {

        try {
            dataFile.close();
            dataReader.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

}
