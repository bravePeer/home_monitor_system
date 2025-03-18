import time
from logger import logger
from monitor_station import MonitorStationDevice, MonitorStationCommunicator
import usb.core
import usb.util
import time
import click
from sensor.sensor import Sensor
from sensor.sensor_builder import SensorBuilder


idVendor=0x0000
idProduct=0x0001


@click.group()
def cli():
    pass

@click.command()
def test_connection():
    logger.info("Testing connection")
    device = MonitorStationDevice()
    device.set_vid_pid(id_vendor=0x0000, id_product=0x0001)
    device.connect()
    test_data = [0x10, 0x00]
    device.write_read(test_data)

@click.command()
def list_usb_devices():
    logger.info("Listing USB devices")
    devices = usb.core.find(find_all=True)
    for device in devices:
        try:
            device_name = usb.util.get_string(device, device.iProduct)
            vid = hex(device.idVendor)
            pid = hex(device.idProduct)
            logger.info(f"Device: {device_name}, VID: {vid}, PID: {pid}")
        except:
            logger.error(f"Cannot get device name for device with VID: {hex(device.idVendor)}, PID: {hex(device.idProduct)}")
            pass

@click.command()
@click.option("--silent/--no-silent", default=True)
def list_sensors(silent):
    logger.silent = silent
    try:
        sensors_count = MonitorStationCommunicator().read_sensors_count()
        click.echo(f"Known {sensors_count} sensors")
    except Exception as e:
        click.echo(f"Cannot read sensors count, error: {e}")

@click.command()
@click.option("--silent/--no-silent", default=True)
def list_sensors_identifiers(silent):
    logger.silent = silent
    try:
        sensors_identifiers = MonitorStationCommunicator().read_sensors_identifiers()
        click.echo(f"Known sensors: {list(map(lambda x: f'0x{x:08X}', sensors_identifiers))}")
    except Exception as e:
        click.echo(f"Cannot read sensors identifiers, error: {e}")

@click.command()
@click.argument("identifier")
@click.option("--silent/--no-silent", default=True)
@click.option("--verbose/--no-verbose", default=False)
def read_sensor_info(identifier, silent, verbose):
    identifier = int(identifier, 0)
    logger.silent = silent
    try:
        sensors_info_raw = MonitorStationCommunicator().read_sensor_info_raw(identifier)
        sensor = Sensor()
        sensor.set_sensor_info(sensors_info_raw)
        click.echo("Sensor info:")
        click.echo(sensor)
        if verbose:
            click.echo(f"Raw info: {sensors_info_raw}")

    except Exception as e:
        click.echo(f"Cannot read sensor info from 0x{identifier:08x}, error: {e}")

@click.command()
@click.argument("identifier")
@click.option("--silent/--no-silent", default=True)
def read_sensor_last_data(identifier, silent):
    identifier = int(identifier, 0)
    logger.silent = silent
    try:
        sensors_info_raw = MonitorStationCommunicator().read_sensor_info_raw(identifier)
        sensor = SensorBuilder().build_from_bytes(sensors_info_raw)
        click.echo("Sensor info:")
        click.echo(sensor)
        

    except Exception as e:
        click.echo(f"Cannot read sensor info from 0x{identifier:08x}, error: {e}")

cli.add_command(test_connection)
cli.add_command(list_usb_devices)
cli.add_command(list_sensors)
cli.add_command(list_sensors_identifiers)
cli.add_command(read_sensor_info)
cli.add_command(read_sensor_last_data)

if __name__ == "__main__":
    logger.init()
    # try:
    #     logger.info("Try to automatically connect to device")
    #     device = MonitorStationDevice()
    #     device.set_vid_pid(id_vendor=0x0000, id_product=0x0001)
    #     device.connect()
    # except:
    #     logger.error("Cannot connect to device")
    cli()