up:
	docker compose down
	docker compose up -d

build-up:
	docker compose down
	docker compose up -d --build

down:
	docker compose down

start:
	docker compose start -d

stop:
	docker compose stop

notebook:
	docker compose exec notebook /bin/sh
