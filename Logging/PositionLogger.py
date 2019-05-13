import logging

# this logger is used only to save the position of the uav

def create_logger():
    logger = logging.getLogger("POSITION ESTIMATION")
    fileHandler = logging.FileHandler("Logging/position_log.csv")
    fileFormatter = logging.Formatter('%(message)s;%(asctime)s,%(msecs)03d', '%H:%M:%S')
    fileHandler.setFormatter(fileFormatter)
    fileHandler.setLevel(logging.DEBUG)
    logger.addHandler(fileHandler)
    logger.setLevel(logging.DEBUG)
    return logger


logger = create_logger()
