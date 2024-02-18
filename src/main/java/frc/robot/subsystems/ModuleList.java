package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;

import com.flybotix.hfr.util.log.ILog;
import com.flybotix.hfr.util.log.Logger;

import frc.common.types.EMatchMode;
import frc.robot.Robot;

public class ModuleList extends Module {
  ILog mLogger = Logger.createLog(ModuleList.class);

  protected List<Module> mModules = new LinkedList<>();

  @Override
  public void modeInit(EMatchMode pMode) {
    mModules.forEach(module -> module.modeInit(pMode));
  }

  @Override
  protected void readInputs() {
    if (Robot.mode() == EMatchMode.TEST) {
      mModules.forEach(
          module -> Robot.CLOCK.report("R-" + module.getClass().getSimpleName(), t -> module.safeReadInputs()));
    } else {
      mModules.forEach(module -> module.safeReadInputs());
    }

  }

  @Override
  protected void setOutputs() {
    if (Robot.mode() == EMatchMode.TEST) {
      mModules.forEach(
          module -> Robot.CLOCK.report("R-" + module.getClass().getSimpleName(), t -> module.safeSetOutputs()));
    } else {
      mModules.forEach(module -> module.safeSetOutputs());
    }
  }

  @Override
  public void shutdown() {
    mModules.forEach(module -> module.shutdown());
  }

  @Override
  public boolean checkModule() {
    boolean allSuccessful = true;
    for (Module module : mModules) {
      boolean moduleSuccessful = module.checkModule();
      allSuccessful = allSuccessful && moduleSuccessful;
      if (!moduleSuccessful) {
        mLogger.error("Self-check failure for module: ", module.getClass());
      } else {
        mLogger.warn("Self-check success for module: ", module.getClass());
      }
    }

    return allSuccessful;
  }

  public void clearModules() {
    mModules.clear();
  }

  public void addModule(WestCoastDrive mWestCoastDrive) {
    if (mWestCoastDrive == null) {
      throw new IllegalArgumentException("the module is null!");
    }

    mModules.add(mWestCoastDrive);
  }
}
