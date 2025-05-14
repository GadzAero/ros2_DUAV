local done = false
local first_run = true
local debug = false  -- Paramètre debug (changer à true ou false pour activer/désactiver les logs)
local relay_index = 0  -- Numéro du relais à surveiller

function update()
    -- Affiche "SCRIPT ALIVE" une seule fois au démarrage
    if first_run then
        gcs:send_text(6, ">>> SCRIPT ALIVE <<<")
        first_run = false
    end

    -- Lire l'état du relais
    local relay_state = relay:get(relay_index)

    -- Vérifier que relay_state n'est pas nil
    if relay_state ~= nil then
        -- Affichage des logs de debug chaque seconde si debug est activé
        if debug then
            gcs:send_text(3, "Relay[" .. relay_index .. "] state: " .. relay_state)
        end

        -- Déclenchement quand le relais est activé
        if relay_state == 1 and not done then
            gcs:send_text(6, ">>> Declenchement via RELAY " .. relay_index .. " <<<")
            gcs:send_text(6, ">>> FIRE POS XX, XX, XX <<")
            done = true
        elseif relay_state == 0 and done then
            done = false  -- Réarmer le déclenchement si le relais est désactivé
        end
    else
        if debug then
            gcs:send_text(3, "Erreur: état du relais non lu.")
        end
    end

    return update, 1000  -- Boucle toutes les 1000 ms (1 seconde)
end

return update()
