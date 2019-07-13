'''
Add a linked Table of Contents to any GitHub flavoured Markdown file!

Author: methylDragon
'''

import sys
import copy
import re

def find_toc_line_no(line_list):
    if type(line_list) is str:
        line_list = line_list.split("\n")

    toc_line_no = -1

    # Note: Line Number starts with index 1
    # This is for ease of understanding wrt. a text editor
    for line_no, line in enumerate(line_list, 1):
        if line.upper().strip() == "## TABLE OF CONTENTS":
            toc_line_no = line_no
            break

    assert toc_line_no > 0, "No table of contents found! Please ensure your Markdown File has a line with ONLY ## Table Of Contents"
    return toc_line_no

def filter_out_code_blocks(line_list, toc_line_no):
    if type(line_list) is str:
        line_list = line_list.split("\n")

    non_code = True
    non_code_line_list = []

    for line_no, line in enumerate(line_list[toc_line_no:], toc_line_no + 1):
        if "```" not in line and non_code:
            non_code_line_list.append((line_no, line))

        elif "```" in line:
            if non_code:
                non_code = False
            elif not non_code:
                non_code = True

    return non_code_line_list

def construct_header_tree(non_code_line_list):
    header_tree = {}
    last_h2 = -1

    for line_no, line in non_code_line_list:
        if line.startswith("#"):
            level = line.count("#")
            text = line[level:].strip()

            if level == 2:
                last_h2 = line_no
                header_tree[last_h2] = {'text': text,'children': []}
            if level == 3:
                header_tree[last_h2]['children'].append((line_no, text))

    return header_tree

def generate_toc_string(header_tree):
    toc_string = ["## Table Of Contents <a name=\"top\"></a>\n\n"]

    for header_no, h2 in enumerate(header_tree.values(), 1):
        toc_string.append("%d. [%s](#%d)    \n" % (header_no, h2['text'], header_no))

        for child_no, child in enumerate(h2['children'], 1):
            toc_string.append("   %d.%d [%s](#%d.%d)    \n" % (header_no, child_no, child[1], header_no, child_no))

    return "".join(toc_string)

def reconstruct_markdown(line_list, toc_line_no, toc_string, header_tree):
    reconstructed_md = copy.copy(line_list)
    reconstructed_md[toc_line_no - 1] = toc_string

    for header_no, h2 in enumerate(header_tree.items(), 1):
        header_line_no = h2[0]
        header_text = h2[1]['text'].strip()

        reconstructed_md[header_line_no - 1] = "## %d. %s <a name=\"%d\"></a>" % (header_no, h2[1]['text'], header_no)

        for child_no, child in enumerate(h2[1]['children'], 1):
            child_line_no = child[0]
            child_text = child[1]

            reconstructed_md[child_line_no - 1] = "### %d.%d %s <a name=\"%d.%d\"></a>\n[go to top](#top)\n" % (header_no, child_no, child_text, header_no, child_no)

    return "\n".join(reconstructed_md)

if __name__ == "__main__":
    input = sys.argv[1]
    try:
        output = sys.argv[2]
    except:
        output_list = input.split("/")
        output_filename = "toc_" + output_list[-1]
        output_list[-1] = output_filename
        output = "/".join(output_list)

    # Read File
    with open(input, "r") as f:
        md = f.read().split("\n")

    toc_line_no = find_toc_line_no(md)
    non_code_md = filter_out_code_blocks(md, toc_line_no)
    header_tree = construct_header_tree(non_code_md)
    toc_string = generate_toc_string(header_tree)

    with open(output, "w") as f:
        f.write(reconstruct_markdown(md, toc_line_no, toc_string, header_tree))
