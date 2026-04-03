from datetime import datetime, timedelta
from typing import Optional, Any
from jose import JWTError, jwt
from passlib.context import CryptContext
import os

# 配置安全参数
DEFAULT_SECRET_KEY = "industrial-grade-secret-change-me"
SECRET_KEY = os.getenv("AGI_WALKER_SECRET_KEY", DEFAULT_SECRET_KEY)
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60 * 24 # 1 天

# Prefer a pure-passlib default to avoid environment-specific bcrypt backend issues,
# while still accepting existing bcrypt hashes during verification/migration.
pwd_context = CryptContext(schemes=["pbkdf2_sha256", "bcrypt"], deprecated="auto")


def _validate_secret_configuration() -> None:
    environment = os.getenv("AGI_WALKER_ENV", "").strip().lower()
    require_explicit_secret = os.getenv("AGI_WALKER_REQUIRE_EXPLICIT_SECRET", "0") == "1"
    is_production = environment in {"prod", "production"}

    if (is_production or require_explicit_secret) and SECRET_KEY == DEFAULT_SECRET_KEY:
        raise RuntimeError(
            "AGI_WALKER_SECRET_KEY must be explicitly configured before starting in production mode."
        )


_validate_secret_configuration()

def verify_password(plain_password: str, hashed_password: str) -> bool:
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def decode_access_token(token: str) -> Optional[dict]:
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except JWTError:
        return None
