import datetime

import colorful


def write_note(s, file):
    d = datetime.datetime.utcnow().replace(microsecond=0)
    s.write('// [note] Auto-generated file\n')
    s.write('// [note] %sZ\n' % (d.isoformat(),))
    s.write('// [note] based on %s\n' % (file,))


def print_generated_file(print_name):
    print('Generate [{}]'.format(colorful.green(print_name)))
