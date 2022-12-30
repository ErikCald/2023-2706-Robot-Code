// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Map;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class ShuffleBoardFaults {

    public enum Section {
        Swerve,
        Mechanisms
    }

    ShuffleboardTab basicDebuggingTab;
    ShuffleboardLayout swerveLayout;
        
    ArrayList <Device> m_devices = new ArrayList<Device>();

    private static ShuffleBoardFaults instance;

    public static ShuffleBoardFaults getInstance() {
        if (instance == null) {
            instance = new ShuffleBoardFaults();
        }
        return instance;
    }
    private ShuffleBoardFaults() {
        basicDebuggingTab = Shuffleboard.getTab("DeviceFaults");

        swerveLayout = basicDebuggingTab.getLayout("SwerveFaults", BuiltInLayouts.kList)
        .withSize(3, 6)
        .withPosition(4, 0)
        .withProperties(Map.of("Label position", "HIDDEN"));
    }

    private class Device {
        Supplier<String> m_getFault;
        String m_deviceName;
        GenericEntry m_entry;

        private Device(Supplier<String> getFault, String deviceName, GenericEntry entry) {
            m_getFault = getFault;
            m_deviceName = deviceName;
            m_entry = entry;
        }

        public boolean equals(Object object) {
            if (object instanceof Device) {
                if (((Device) object).m_deviceName == this.m_deviceName) {
                    return true;
                }
            }
            return false;
        }
    }

    private ShuffleboardLayout getLayout(Section section) {
        if (section == Section.Swerve) {
            return swerveLayout;
        }

        return swerveLayout;
        
    }
    public ShuffleBoardFaults addREVDevice(Supplier<Short> getFault, String deviceName, Section section) {
        m_devices.add(new Device(
            () -> getRevString(getFault.get()),
            deviceName,
            getLayout(section).add(deviceName, deviceName + ": FaultsNotCheckedYet").getEntry()
        ));
        return this;
    }

    private String getRevString(Short faultID) {
        if (faultID < 0 ) {
            return "";
        }
        FaultID id = CANSparkMax.FaultID.fromId(faultID);

        if (id == null) {
            return "";
        }
        return id.toString();
    }



    public void update() {
        for (int i = 0; i < m_devices.size(); i++) {
            Device dev = m_devices.get(i);
            String msg = dev.m_getFault.get();
            if (msg == "") {
                msg = "Ok";
            }
            dev.m_entry.setString(dev.m_deviceName + ": " + msg);
        }
    }


}
