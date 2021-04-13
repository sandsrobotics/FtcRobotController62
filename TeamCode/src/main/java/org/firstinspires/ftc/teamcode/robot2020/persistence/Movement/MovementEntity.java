package org.firstinspires.ftc.teamcode.robot2020.persistence.Movement;

import androidx.room.ColumnInfo;
import androidx.room.Entity;
import androidx.room.Ignore;
import androidx.room.PrimaryKey;

@Entity(tableName = "Movement")
public class MovementEntity {

    @PrimaryKey (autoGenerate = true)
    public int id;

    @ColumnInfo(name = "name")
    public String name;
    @ColumnInfo(name = "motor_id")
    public int motor_id;
    @ColumnInfo(name = "motor_tick")
    public float motor_tick;

    @Ignore
    public MovementEntity() { }

    public MovementEntity(String name, int motor_id, float motor_tick) {
        this.name = name;
        this.motor_id = motor_id;
        this.motor_tick = motor_tick;
    }
}
