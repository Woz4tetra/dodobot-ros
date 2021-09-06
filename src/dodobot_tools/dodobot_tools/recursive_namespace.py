from types import SimpleNamespace
from collections.abc import Iterable


class RecursiveNamespace(SimpleNamespace):
    """
    Similar to types.SimpleNamespace, except nested dictionaries become RecursiveNamespace too
    """

    @staticmethod
    def map_entry(entry):
        if isinstance(entry, dict):
            return RecursiveNamespace(**entry)
        return entry

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.update(kwargs)

    def __iter__(self):
        for attr_name, attr_val in self.__dict__.items():
            if isinstance(attr_val, RecursiveNamespace):
                yield attr_name, dict(attr_val)
            elif type(attr_val) == list or type(attr_val) == tuple:
                result = []
                for list_attr in attr_val:
                    if isinstance(list_attr, RecursiveNamespace):
                        result.append(dict(list_attr))
                    else:
                        result.append(list_attr)
                yield attr_name, result
            else:
                yield attr_name, attr_val

    def items(self):
        return self.__dict__.items()
    
    def keys(self):
        return self.__dict__.keys()
    
    def values(self):
        return self.__dict__.values()

    def update(self, d: dict):
        for key, val in d.items():
            if type(val) == dict:
                setattr(self, key, RecursiveNamespace(**val))
            elif type(val) == list or type(val) == tuple:
                setattr(self, key, list(map(self.map_entry, val)))
            else:
                setattr(self, key, val)

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        return setattr(self, key, value)


if __name__ == '__main__':
    import pprint


    def test():
        test_dict = {
            "A": {"A_A": 0, "A_B": 1, "A_C": [{"A_C_A": 2.0}, 2.1, 2.2]},
            "B": {"B_A": 0, "B_B": 1, "B_C": [{"B_C_A": 2.0}, 2.1, 2.2], "B_D": 3},
            "C": {"C_A": 0, "C_B": 1, "C_C": [{"C_C_A": 2.0}, 2.1, 2.2], "C_D": 3, "C_E": 4, "C_F": "567"},
        }
        ns = RecursiveNamespace(**test_dict)
        pprint.pprint(test_dict)
        pprint.pprint(dict(ns))
        assert dict(ns) == test_dict, "%s != %s" % (dict(ns), test_dict)
        assert ns.A.A_A == 0
        assert ns.A.A_C[0].A_C_A == 2.0
        assert ns.A.A_C[1] == 2.1
        assert ns.B.B_B == 1
        assert ns.C.C_E == 4
        assert ns["A"]["A_A"] == 0
        assert ns["A"]["A_C"][0]["A_C_A"] == 2.0
        assert ns["A"]["A_C"][1] == 2.1
        assert ns["B"]["B_B"] == 1
        assert ns["C"]["C_E"] == 4
        assert ns["C"]["C_F"] == "567"


    test()
