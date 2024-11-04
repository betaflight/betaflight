from jinja2 import Environment, FileSystemLoader
import jinja2
import os
from datetime import datetime
import pprint

# generate normalization preprocessor file for serial ports

# configuration for template generation
serials = {
    "UART": {"ids": [i + 1 for i in range(10)],
             "inverter": True,
             },
    "LPUART": {"ids": [1],
               "depends": {"UART"},
#               "inverter": True,   # TODO: old code compatibility only, disabled
               },
    "SOFTSERIAL": {"ids": [i + 1 for i in range(2)],
                   "use_enables_all": True,
                   "force_continuous": True,
                   },
    "VCP": {"singleton": True,
            "no_pins": True}
}

def flatten_config(data):
    flattened_list = []
    for key, value in data.items():
        flattened_dict = {'typ': key}
        # Update this new dictionary with the inner dictionary's items
        flattened_dict.update(value)
        flattened_list.append(flattened_dict)
    return flattened_list

def pprint_filter(value, indent=4):
    return pprint.pformat(value, indent=indent)

def rdepends_filter(cfg, typ):
    return list(set([ c['typ'] for c in cfg if typ in c['depends'] ]))

def main():
    # Setup Jinja2 environment
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Set the template directory relative to the current script
    template_dir = os.path.join(current_dir, 'templates')
    env = Environment(loader=FileSystemLoader(template_dir),
                      autoescape=True,
                      undefined=jinja2.StrictUndefined,  # Throws an error on undefined variables
                      trim_blocks = True,
                      )
    env.filters['pprint'] = pprint_filter
    env.filters['zip'] = zip
    env.filters['rdepends'] = rdepends_filter
    template = env.get_template('serial_post.h')

    # Read license file
    license_path = 'DEFAULT_LICENSE.md'
    with open(license_path, 'r') as file:
        t_license = file.read()

    context = {}
    context['license'] = t_license
    context['date_generated'] = datetime.now().strftime('%Y-%m-%d')
    context['user_config'] = serials
    config = flatten_config(serials)
    context['config'] = config

    for cfg in config:
        singleton = cfg.setdefault('singleton', False)
        no_pins = cfg.setdefault('no_pins', False)
        inverter = cfg.setdefault('inverter', False)
        cfg.setdefault("use_enables_all", False)
        cfg.setdefault("force_continuous", False)
        cfg.setdefault("depends", {})
        typ = cfg['typ']
        if singleton:
            cfg['ports']= [ f"{typ}" ]
        else:
            cfg['ports'] = [ f"{typ}{i}" for i in cfg["ids"] ]

    # Render the template with the license content
    output = template.render(**context)

    if False:
        # Output or save to a file
        output_path = './src/main/target/serial_post.h'
        with open(output_path, 'w') as file:
            file.write(output)
        print(f"Generated file saved to {output_path}")
    else:
        print(output)

if __name__ == '__main__':
    main()
