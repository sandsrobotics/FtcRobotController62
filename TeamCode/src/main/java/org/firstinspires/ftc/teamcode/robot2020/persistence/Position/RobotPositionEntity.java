package org.firstinspires.ftc.teamcode.robot2020.persistence.Position;

import androidx.room.ColumnInfo;
import androidx.room.Entity;
import androidx.room.Ignore;
import androidx.room.PrimaryKey;

@Entity(tableName = "PositionTracker")
public class RobotPositionEntity
{
    @PrimaryKey(autoGenerate = true)
    public int id;

    @ColumnInfo(name = "time of entry")
    public float entryMs;
    @ColumnInfo(name = "run number")
    public int runNumber;
    @ColumnInfo(name = "pos X")
    public double posX;
    @ColumnInfo(name = "pos Y")
    public double posY;
    @ColumnInfo(name = "rotation")
    public double rotation;
    @ColumnInfo(name = "accuracy")
    public double accuracy;

    @Ignore
    public RobotPositionEntity() { }

    public RobotPositionEntity(int runNumber, double posX, double posY, double rotation, double accuracy)
    {
        this.runNumber = runNumber;
        this.entryMs = System.currentTimeMillis();
        this.posX = posX;
        this.posY = posY;
        this.rotation = rotation;
        this.accuracy = accuracy;
    }
}
