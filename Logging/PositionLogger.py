import logging


def create_logger():
    logger = logging.getLogger("POSITION ESTIMATION")
    fileHandler = logging.FileHandler("Logging/postion_log.txt")
    fileFormatter = logging.Formatter('%(asctime)s.%(msecs)03d, %(message)s', "%H:%M:%S")
    fileHandler.setFormatter(fileFormatter)
    fileHandler.setLevel(logging.DEBUG)
    logger.addHandler(fileHandler)
    logger.setLevel(logging.DEBUG)
    return logger


logger = create_logger()
