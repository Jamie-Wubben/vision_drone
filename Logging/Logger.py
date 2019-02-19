import logging


def create_logger():
    logger = logging.getLogger("DRONE VISION")
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - [DRONE VISION] %(levelname)s - %(message)s')
    ch.setFormatter(formatter)
    logger.addHandler(ch)

    fileHandler = logging.FileHandler("Logging/vision_log.txt")
    fileFormatter = logging.Formatter('%(asctime)s - [DRONE VISION] %(levelname)s - %(funcName)s - %(message)s')
    fileHandler.setFormatter(fileFormatter)
    fileHandler.setLevel(logging.DEBUG)
    logger.addHandler(fileHandler)
    logger.setLevel(logging.DEBUG)
    return logger


logger = create_logger()