#!/usr/bin/env python3
"""Fix compile_commands.json for VS Code IntelliSense.

1. Replace .arduino/build/sketch paths with workspace source paths.
2. Expand @flags/includes response files into explicit -I flags so
   VS Code IntelliSense can resolve all ESP-IDF headers.
"""
import json
import os
import re

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CC_JSON = os.path.join(SCRIPT_DIR, 'compile_commands.json')


def fix_sketch_paths(content):
    bs = chr(92)
    pat = bs * 2 + '.arduino' + bs * 2 + 'build' + bs * 2 + 'sketch'
    n = content.count(pat)
    return content.replace(pat, ''), n


def expand_args(args):
    """Expand -iprefix + @flags/includes into explicit -I flags."""
    new_args = []
    iprefix = None
    i = 0
    while i < len(args):
        arg = args[i]

        if arg == '-iprefix' and i + 1 < len(args):
            iprefix = args[i + 1].rstrip('/\\')
            new_args.extend([arg, args[i + 1]])
            i += 2
            continue

        if arg.startswith('@') and 'flags/includes' in arg:
            rsp = arg[1:].replace('/', os.sep)
            if os.path.isfile(rsp) and iprefix:
                with open(rsp, 'r') as f:
                    rsp_content = f.read()
                for rel in re.findall(r'-iwithprefixbefore\s+(\S+)', rsp_content):
                    full = iprefix + os.sep + rel.replace('/', os.sep)
                    new_args.append('-I' + full)
            i += 1
            continue

        new_args.append(arg)
        i += 1

    return new_args


def main():
    with open(CC_JSON, 'r', encoding='utf-8') as f:
        content = f.read()

    content, n_paths = fix_sketch_paths(content)
    data = json.loads(content)

    n_expanded = 0
    for entry in data:
        if 'arguments' not in entry:
            continue
        has_rsp = any('flags/includes' in a for a in entry['arguments'])
        entry['arguments'] = expand_args(entry['arguments'])
        if has_rsp:
            n_expanded += 1

    with open(CC_JSON, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=4, ensure_ascii=False)

    print(f'Done: {n_paths} sketch paths fixed, '
          f'{n_expanded}/{len(data)} entries expanded')


if __name__ == '__main__':
    main()
