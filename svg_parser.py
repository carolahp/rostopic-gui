from xml.etree.ElementTree import parse, ElementTree

def parse(filename):
    tree = ElementTree(file=filename)
    return tree

def print_hierarchy(el):
    for e in el:
        print_hierarchy(e)

def get_class(class_name, el, dic):
    if el.tag.rstrip('\n').rstrip('\r').endswith("}g"):
        if el.attrib["class"] == class_name:
            dic[el.attrib["id"]] = el       
    for e in el:
        get_class(class_name, e, dic)

def main():
    xml = parse("test.svg")
    root = xml.getroot()
    #print_hierarchy(root)
    dic = {}
    get_class("edge", root, dic)
    for e in dic.values():
        for child in e:
            if child.tag.rstrip('\n').rstrip('\r').endswith("}polygon"):
                child.attrib["stroke"] = "red"
                child.attrib["fill"] = "red"
                child.attrib["stroke-width"] = "6"
            elif child.tag.rstrip('\n').rstrip('\r').endswith("}path"):
                child.attrib["stroke"] = "red"
                child.attrib["stroke-width"] = "6"
    xml.write("caro.svg")

if __name__ == "__main__":
    main()
