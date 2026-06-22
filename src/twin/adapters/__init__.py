from src.twin.adapters.base import TwinAdapter
from src.twin.adapters.custom import CustomAdapter
from src.twin.adapters.internal import InternalAdapter
from src.twin.adapters.pybullet import PyBulletAdapter
from src.twin.adapters.webots import WebotsAdapter

_custom = CustomAdapter()

_ADAPTERS: dict[str, TwinAdapter] = {
    "internal": InternalAdapter(),
    "generic": InternalAdapter(),
    "custom": _custom,
    "butlerbot": _custom,
    "webots": WebotsAdapter(),
    "pybullet": PyBulletAdapter(),
}


def get_adapter(name: str) -> TwinAdapter:
    return _ADAPTERS.get(name, _ADAPTERS["generic"])


def registered_adapters() -> list[str]:
    return sorted(_ADAPTERS.keys())