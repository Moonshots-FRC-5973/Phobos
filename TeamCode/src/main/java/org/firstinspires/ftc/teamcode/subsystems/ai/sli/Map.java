package org.firstinspires.ftc.teamcode.subsystems.ai.sli;

import java.util.ArrayList;

public class Map {
    private ArrayList<String> keys = new ArrayList<String>();
    private ArrayList<Object> values = new ArrayList<Object>();

    public void addValue(String key, Object value) {
        keys.add(key);
        values.add(value);
    }

    public Object getValue(String key) {
        try {
            return values.get(keys.indexOf(key));
        } catch(Exception e) {
            return null;
        }
    }
}
