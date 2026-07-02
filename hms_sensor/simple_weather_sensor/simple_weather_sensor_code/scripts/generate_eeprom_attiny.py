import click

@click.command("generate-eeprom")
@click.option('--radio-address', default="0x62636430", help='Radio address.')
@click.option('--sensor-id', default="", help='.')
@click.option('--soft-ver', default="", help='.')
@click.option('--hard-ver', default="", help='.')
@click.option('--sensor-type', default="", help='.')
@click.option('--max-data-count', default="", help='.')
@click.option('--max-calib-count', default="", help='.')
@click.option('--r1-val', default="", help='.')
@click.option('--r2-val', default="", help='.')

def generate_eeprom(count):
    for x in range(count):
        click.echo("Hello!")

@click.group()
def cli():
    pass

cli.add_command(generate_eeprom)

if __name__ == "__main__":
    cli()