import sys
import re
from jinja2 import Template

# Jinja2 template for the header file
template_content = """
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

{% for handler in exception_handlers %}
void {{ handler }}_Handler(void);
{% endfor %}

{% for handler in irq_handlers %}
void {{ handler }}_IRQHandler(void);
{% endfor %}

#ifdef __cplusplus
}
#endif
"""

# Regex to match IRQ handlers
irq_regex = re.compile(r"\.weak\s+(\w+)_IRQHandler")
exception_regex = re.compile(r"\.weak\s+(\w+)_Handler")

# List to store IRQ handlers
irq_handlers = set()
exception_handlers = set()

# Read the input from stdin
for line in sys.stdin:
    if match := irq_regex.search(line):
        irq_handlers.add(match.group(1))
    if match := exception_regex.search(line):
        exception_handlers.add(match.group(1))

# Create the Jinja2 template
template = Template(template_content, trim_blocks=True)

# Render the template with the IRQ handlers
output = template.render(exception_handlers=sorted(exception_handlers), irq_handlers=sorted(irq_handlers))

# Output the result to stdout
sys.stdout.write(output)
